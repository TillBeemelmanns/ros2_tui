//! Inline images: the kitty graphics protocol where the terminal supports it
//! (kitty, Ghostty, WezTerm, ...), unicode half blocks everywhere else.

use base64::Engine;
use flate2::{write::ZlibEncoder, Compression};
use image::RgbImage;
use ratatui::{buffer::Buffer, layout::Rect, style::Color, widgets::Widget};
use std::io::{self, Write};
use std::time::{Duration, Instant};

/// Environment variable forcing a protocol: `kitty` or `halfblocks`.
pub const GRAPHICS_ENV: &str = "ROS2_TUI_GRAPHICS";

/// Upper bound on the pixels sent per kitty frame. The terminal scales the
/// image to its cells, so more pixels only add detail, at the cost of
/// bandwidth that matters over SSH.
const KITTY_MAX_PIXELS: u32 = 960 * 720;

const KITTY_CHUNK: usize = 4096;
const QUERY_TIMEOUT: Duration = Duration::from_millis(1000);

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum GraphicsProtocol {
    Kitty,
    Halfblocks,
}

#[derive(Debug, Clone, Copy)]
pub struct TerminalGraphics {
    pub protocol: GraphicsProtocol,
    /// Size of one cell in pixels.
    pub cell_width: u16,
    pub cell_height: u16,
}

impl Default for TerminalGraphics {
    fn default() -> Self {
        Self {
            protocol: GraphicsProtocol::Halfblocks,
            cell_width: 10,
            cell_height: 20,
        }
    }
}

impl TerminalGraphics {
    pub fn protocol_name(&self) -> &'static str {
        match self.protocol {
            GraphicsProtocol::Kitty => "kitty graphics",
            GraphicsProtocol::Halfblocks => "half blocks",
        }
    }

    /// Pixel size images should be decoded to for display in `area`.
    pub fn target_pixels(&self, area: Rect) -> (u32, u32) {
        match self.protocol {
            GraphicsProtocol::Kitty => {
                let w = area.width as u32 * self.cell_width as u32;
                let h = area.height as u32 * self.cell_height as u32;
                let pixels = (w * h).max(1);
                if pixels <= KITTY_MAX_PIXELS {
                    (w.max(1), h.max(1))
                } else {
                    let scale = (KITTY_MAX_PIXELS as f64 / pixels as f64).sqrt();
                    (
                        ((w as f64 * scale) as u32).max(1),
                        ((h as f64 * scale) as u32).max(1),
                    )
                }
            }
            // Two pixels per cell, stacked vertically.
            GraphicsProtocol::Halfblocks => {
                (area.width.max(1) as u32, area.height.max(1) as u32 * 2)
            }
        }
    }

    /// Pixel size to render a point cloud at to fill `area`, and the width
    /// over height of one of those pixels. Without kitty graphics, clouds are
    /// drawn with braille dots, 2x4 per cell.
    pub fn cloud_pixels(&self, area: Rect) -> (u32, u32, f32) {
        match self.protocol {
            GraphicsProtocol::Kitty => {
                let (w, h) = self.target_pixels(area);
                (w, h, 1.0)
            }
            GraphicsProtocol::Halfblocks => (
                area.width.max(1) as u32 * 2,
                area.height.max(1) as u32 * 4,
                (self.cell_width as f32 / 2.0) / (self.cell_height as f32 / 4.0),
            ),
        }
    }

    /// Cells an image of `width` x `height` pixels covers when scaled to fit
    /// `area`, centered in it.
    pub fn placement(&self, area: Rect, width: u32, height: u32) -> Rect {
        if width == 0 || height == 0 || area.width == 0 || area.height == 0 {
            return Rect::new(area.x, area.y, 0, 0);
        }
        let (cell_w, cell_h) = match self.protocol {
            GraphicsProtocol::Kitty => (self.cell_width as f64, self.cell_height as f64),
            GraphicsProtocol::Halfblocks => (1.0, 2.0),
        };
        let scale = (area.width as f64 * cell_w / width as f64)
            .min(area.height as f64 * cell_h / height as f64);
        let cols = ((width as f64 * scale / cell_w).round() as u16).clamp(1, area.width);
        let rows = ((height as f64 * scale / cell_h).round() as u16).clamp(1, area.height);
        Rect::new(
            area.x + (area.width - cols) / 2,
            area.y + (area.height - rows) / 2,
            cols,
            rows,
        )
    }
}

/// Detect the graphics protocol and cell size. Must run in raw mode, before
/// anything else reads terminal input: it reads the terminal's answers to a
/// kitty graphics query, a cell size query and a device attributes query,
/// which every terminal answers, so it does not wait out the timeout.
pub fn detect() -> TerminalGraphics {
    let mut graphics = TerminalGraphics::default();
    let forced = std::env::var(GRAPHICS_ENV).unwrap_or_default();
    if forced == "halfblocks" {
        return graphics;
    }

    let reply = match query_terminal("\x1b_Gi=31,s=1,v=1,a=q,t=d,f=24;AAAA\x1b\\\x1b[16t\x1b[c") {
        Ok(reply) => reply,
        Err(e) => {
            crate::debug_log(&format!("Terminal graphics query failed: {}", e));
            return graphics;
        }
    };
    crate::debug_log(&format!("Terminal graphics reply: {:?}", reply));

    if let Some((height, width)) = parse_cell_size(&reply) {
        graphics.cell_width = width;
        graphics.cell_height = height;
    }
    // tmux swallows the graphics escapes unless passed through, so a reply
    // from the outer terminal does not mean images will show.
    let in_tmux = std::env::var_os("TMUX").is_some();
    if forced == "kitty" || (reply.contains("\x1b_Gi=31;OK") && !in_tmux) {
        graphics.protocol = GraphicsProtocol::Kitty;
    }
    graphics
}

/// `CSI 6 ; height ; width t`, the reply to `CSI 16 t`.
fn parse_cell_size(reply: &str) -> Option<(u16, u16)> {
    let start = reply.find("\x1b[6;")? + 4;
    let end = start + reply[start..].find('t')?;
    let mut parts = reply[start..end].split(';');
    let height = parts.next()?.parse().ok()?;
    let width = parts.next()?.parse().ok()?;
    (height > 0 && width > 0).then_some((height, width))
}

#[cfg(unix)]
fn query_terminal(query: &str) -> io::Result<String> {
    let mut out = io::stdout();
    out.write_all(query.as_bytes())?;
    out.flush()?;

    // Read fd 0 directly: std's buffered stdin could keep bytes past the
    // reply, which the event loop would then never see.
    let deadline = Instant::now() + QUERY_TIMEOUT;
    let mut reply = Vec::new();
    let mut buf = [0u8; 256];
    loop {
        let remaining = deadline.saturating_duration_since(Instant::now());
        if remaining.is_zero() {
            break;
        }
        let mut fds = libc::pollfd {
            fd: 0,
            events: libc::POLLIN,
            revents: 0,
        };
        // SAFETY: `fds` is one valid pollfd and `buf` is a valid buffer of
        // the length passed.
        let ready = unsafe { libc::poll(&mut fds, 1, remaining.as_millis() as libc::c_int) };
        if ready <= 0 {
            break;
        }
        let n = unsafe { libc::read(0, buf.as_mut_ptr().cast(), buf.len()) };
        if n <= 0 {
            break;
        }
        reply.extend_from_slice(&buf[..n as usize]);
        if contains_device_attributes(&reply) {
            break;
        }
    }
    Ok(String::from_utf8_lossy(&reply).into_owned())
}

#[cfg(not(unix))]
fn query_terminal(_query: &str) -> io::Result<String> {
    Err(io::Error::new(
        io::ErrorKind::Unsupported,
        "not a unix terminal",
    ))
}

/// Whether `reply` holds the primary device attributes answer, `CSI ? ... c`,
/// which terminals send after the replies to the queries before it.
fn contains_device_attributes(reply: &[u8]) -> bool {
    reply.windows(3).enumerate().any(|(i, w)| {
        w == b"\x1b[?"
            && reply[i + 3..]
                .iter()
                .find(|b| !(b.is_ascii_digit() || **b == b';'))
                .is_some_and(|b| *b == b'c')
    })
}

/// The image currently shown through the kitty protocol.
#[derive(Debug, Default)]
pub struct KittyPlacement {
    shown: Option<Shown>,
    /// Bytes written over the last second, for the bandwidth readout.
    sent: Vec<(Instant, usize)>,
}

#[derive(Debug)]
struct Shown {
    id: u32,
    frame_seq: u64,
    rect: Rect,
}

impl KittyPlacement {
    /// Show frame `frame_seq` (`image`) in `rect`, unless it already is.
    /// A new frame goes up under the other of two image ids before the old
    /// one is deleted, so the picture never flickers.
    pub fn show(
        &mut self,
        out: &mut impl Write,
        image: &RgbImage,
        frame_seq: u64,
        rect: Rect,
        force: bool,
    ) -> io::Result<()> {
        if let Some(shown) = &self.shown {
            if !force && shown.frame_seq == frame_seq && shown.rect == rect {
                return Ok(());
            }
        }
        let old_id = self.shown.as_ref().map(|s| s.id);
        let id = if old_id == Some(1) { 2 } else { 1 };
        let bytes = write_kitty_image(out, image, id, rect)?;
        if let Some(old_id) = old_id {
            write_kitty_delete(out, old_id)?;
        }
        out.flush()?;
        self.shown = Some(Shown {
            id,
            frame_seq,
            rect,
        });
        let now = Instant::now();
        self.sent.push((now, bytes));
        self.sent
            .retain(|(at, _)| now.duration_since(*at) < Duration::from_secs(1));
        Ok(())
    }

    /// Remove the shown image, if any.
    pub fn clear(&mut self, out: &mut impl Write) -> io::Result<()> {
        if let Some(shown) = self.shown.take() {
            write_kitty_delete(out, shown.id)?;
            out.flush()?;
        }
        self.sent.clear();
        Ok(())
    }

    pub fn is_showing(&self) -> bool {
        self.shown.is_some()
    }

    /// Bytes per second sent to the terminal, over the last second.
    pub fn bytes_per_second(&self) -> usize {
        let now = Instant::now();
        self.sent
            .iter()
            .filter(|(at, _)| now.duration_since(*at) < Duration::from_secs(1))
            .map(|(_, n)| n)
            .sum()
    }
}

/// Transmit `image` zlib-compressed and display it scaled to `rect`.
/// Returns the number of bytes written.
fn write_kitty_image(
    out: &mut impl Write,
    image: &RgbImage,
    id: u32,
    rect: Rect,
) -> io::Result<usize> {
    let mut encoder = ZlibEncoder::new(Vec::new(), Compression::fast());
    encoder.write_all(image.as_raw())?;
    let payload = base64::engine::general_purpose::STANDARD.encode(encoder.finish()?);

    let mut seq = Vec::with_capacity(payload.len() + 256);
    // Save the cursor, move it to the top-left cell of the placement.
    write!(seq, "\x1b7\x1b[{};{}H", rect.y + 1, rect.x + 1)?;
    let chunks: Vec<&[u8]> = payload.as_bytes().chunks(KITTY_CHUNK).collect();
    for (i, chunk) in chunks.iter().enumerate() {
        let more = u8::from(i + 1 < chunks.len());
        if i == 0 {
            // q=2: no replies, they would arrive as keyboard input.
            // C=1: leave the cursor where it is.
            write!(
                seq,
                "\x1b_Ga=T,f=24,o=z,t=d,i={},p=1,s={},v={},c={},r={},C=1,q=2,m={};",
                id,
                image.width(),
                image.height(),
                rect.width,
                rect.height,
                more
            )?;
        } else {
            write!(seq, "\x1b_Gm={};", more)?;
        }
        seq.extend_from_slice(chunk);
        seq.extend_from_slice(b"\x1b\\");
    }
    seq.extend_from_slice(b"\x1b8");
    out.write_all(&seq)?;
    Ok(seq.len())
}

fn write_kitty_delete(out: &mut impl Write, id: u32) -> io::Result<()> {
    write!(out, "\x1b_Ga=d,d=I,i={},q=2\x1b\\", id)
}

/// Draws an image with `▀`: the foreground colors the top pixel of a cell,
/// the background the bottom one. The image must already be sized to at
/// most the area width and twice its height.
pub struct Halfblocks<'a>(pub &'a RgbImage);

impl Widget for Halfblocks<'_> {
    fn render(self, area: Rect, buf: &mut Buffer) {
        let image = self.0;
        let (width, height) = image.dimensions();
        let cols = (width as u16).min(area.width);
        let rows = (height.div_ceil(2) as u16).min(area.height);
        let x0 = area.x + (area.width - cols) / 2;
        let y0 = area.y + (area.height - rows) / 2;
        let color = |x: u32, y: u32| {
            let [r, g, b] = image.get_pixel(x, y).0;
            Color::Rgb(r, g, b)
        };
        for row in 0..rows {
            for col in 0..cols {
                let (x, y) = (col as u32, row as u32 * 2);
                let bottom = if y + 1 < height {
                    color(x, y + 1)
                } else {
                    Color::Reset
                };
                buf[(x0 + col, y0 + row)]
                    .set_char('▀')
                    .set_fg(color(x, y))
                    .set_bg(bottom);
            }
        }
    }
}

/// Draws an image with braille dots, 2x4 per cell: a dot for every pixel
/// that is not black, the cell colored like its brightest dot. Suits sparse
/// images such as point clouds, at four times the resolution of half blocks.
/// The image must be sized to twice the area width and four times its height.
pub struct Braille<'a>(pub &'a RgbImage);

impl Widget for Braille<'_> {
    fn render(self, area: Rect, buf: &mut Buffer) {
        // Dot bits of the braille block by position within the cell.
        const BITS: [[u32; 2]; 4] = [[0x01, 0x08], [0x02, 0x10], [0x04, 0x20], [0x40, 0x80]];
        let image = self.0;
        let (width, height) = image.dimensions();
        for row in 0..area.height.min((height / 4) as u16) {
            for col in 0..area.width.min((width / 2) as u16) {
                let mut bits = 0;
                let mut brightest = [0u8; 3];
                for (dy, row_bits) in BITS.iter().enumerate() {
                    for (dx, bit) in row_bits.iter().enumerate() {
                        let px = image
                            .get_pixel(col as u32 * 2 + dx as u32, row as u32 * 4 + dy as u32)
                            .0;
                        if px != [0, 0, 0] {
                            bits |= bit;
                            let sum = |p: [u8; 3]| p.iter().map(|&c| c as u32).sum::<u32>();
                            if sum(px) > sum(brightest) {
                                brightest = px;
                            }
                        }
                    }
                }
                if bits != 0 {
                    let [r, g, b] = brightest;
                    buf[(area.x + col, area.y + row)]
                        .set_char(char::from_u32(0x2800 + bits).unwrap())
                        .set_fg(Color::Rgb(r, g, b));
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_cell_size_reply() {
        let reply = "\x1b_Gi=31;OK\x1b\\\x1b[6;38;17t\x1b[?62;22c";
        assert_eq!(parse_cell_size(reply), Some((38, 17)));
        assert_eq!(parse_cell_size("\x1b[?62c"), None);
    }

    #[test]
    fn recognizes_device_attributes_reply() {
        assert!(contains_device_attributes(
            b"\x1b_Gi=31;OK\x1b\\\x1b[?62;22c"
        ));
        assert!(!contains_device_attributes(
            b"\x1b_Gi=31;OK\x1b\\\x1b[?62;2"
        ));
    }

    #[test]
    fn kitty_target_is_capped() {
        let graphics = TerminalGraphics {
            protocol: GraphicsProtocol::Kitty,
            cell_width: 20,
            cell_height: 40,
        };
        let (w, h) = graphics.target_pixels(Rect::new(0, 0, 200, 50));
        assert!(w * h <= KITTY_MAX_PIXELS);
        assert_eq!(w / h, 4000 / 2000);
        let (w, h) = graphics.target_pixels(Rect::new(0, 0, 10, 5));
        assert_eq!((w, h), (200, 200));
    }

    #[test]
    fn placement_fits_and_centers() {
        let graphics = TerminalGraphics {
            protocol: GraphicsProtocol::Kitty,
            cell_width: 10,
            cell_height: 20,
        };
        // 16:9 in an area 100x20 cells (1000x400 px): height bound.
        let rect = graphics.placement(Rect::new(0, 1, 100, 20), 1600, 900);
        assert_eq!((rect.width, rect.height), (71, 20));
        assert_eq!((rect.x, rect.y), (14, 1));
    }

    #[test]
    fn kitty_image_is_chunked_with_continuation_flags() {
        let image = RgbImage::from_fn(64, 64, |x, y| image::Rgb([x as u8 * 4, y as u8 * 4, 7]));
        let mut out = Vec::new();
        write_kitty_image(&mut out, &image, 1, Rect::new(2, 3, 8, 4)).unwrap();
        let text = String::from_utf8(out).unwrap();
        assert!(text.starts_with("\x1b7\x1b[4;3H\x1b_Ga=T,f=24,o=z"));
        assert!(text.contains("s=64,v=64,c=8,r=4"));
        assert!(text.ends_with("\x1b\\\x1b8"));
        assert_eq!(text.matches("m=0;").count(), 1);
    }

    #[test]
    fn braille_sets_dots_for_lit_pixels() {
        let mut image = RgbImage::new(2, 4);
        image.put_pixel(0, 0, image::Rgb([10, 10, 10]));
        image.put_pixel(1, 3, image::Rgb([200, 0, 0]));
        let mut buf = Buffer::empty(Rect::new(0, 0, 1, 1));
        Braille(&image).render(buf.area, &mut buf);
        let cell = &buf[(0, 0)];
        assert_eq!(cell.symbol(), "\u{2881}");
        assert_eq!(cell.fg, Color::Rgb(200, 0, 0));
    }

    #[test]
    fn halfblocks_pack_two_pixels_per_cell() {
        let image = RgbImage::from_fn(2, 2, |_, y| image::Rgb([y as u8 * 100, 0, 0]));
        let mut buf = Buffer::empty(Rect::new(0, 0, 2, 1));
        Halfblocks(&image).render(buf.area, &mut buf);
        let cell = &buf[(0, 0)];
        assert_eq!(cell.symbol(), "▀");
        assert_eq!(cell.fg, Color::Rgb(0, 0, 0));
        assert_eq!(cell.bg, Color::Rgb(100, 0, 0));
    }
}
