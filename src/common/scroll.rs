//! Scrolling of the topic and parameter tables.

use ratatui::{
    layout::Rect,
    widgets::{Scrollbar, ScrollbarOrientation, ScrollbarState},
    Frame,
};

/// Rows a bordered table with a one-line header has for items.
pub fn table_rows(area: Rect) -> usize {
    area.height.saturating_sub(3) as usize
}

/// The first item to show. Moves `offset` only as far as needed to keep
/// `selected` on screen, and pulls it back when the list shrank, so the
/// table never shows empty rows while items above are hidden.
pub fn scroll_offset(offset: usize, selected: usize, len: usize, rows: usize) -> usize {
    if rows == 0 {
        return 0;
    }
    let offset = offset.min(len.saturating_sub(rows));
    if selected < offset {
        selected
    } else if selected >= offset + rows {
        selected + 1 - rows
    } else {
        offset
    }
}

/// Draws a scrollbar on the right border of a bordered table with a header,
/// if the items do not fit.
pub fn render_table_scrollbar(f: &mut Frame, area: Rect, len: usize, offset: usize) {
    let rows = table_rows(area);
    if len <= rows {
        return;
    }
    let track = Rect {
        y: area.y + 2,
        height: rows as u16,
        ..area
    };
    let mut state = ScrollbarState::new(len - rows + 1)
        .viewport_content_length(rows)
        .position(offset);
    let scrollbar = Scrollbar::new(ScrollbarOrientation::VerticalRight)
        .begin_symbol(None)
        .end_symbol(None)
        .track_symbol(Some("│"))
        .thumb_symbol("█");
    f.render_stateful_widget(scrollbar, track, &mut state);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn keeps_the_offset_while_the_selection_is_on_screen() {
        assert_eq!(scroll_offset(10, 10, 100, 30), 10);
        assert_eq!(scroll_offset(10, 39, 100, 30), 10);
    }

    #[test]
    fn moves_only_as_far_as_needed() {
        assert_eq!(scroll_offset(10, 40, 100, 30), 11);
        assert_eq!(scroll_offset(10, 9, 100, 30), 9);
        // Wrapping around from the last item to the first, and back.
        assert_eq!(scroll_offset(70, 0, 100, 30), 0);
        assert_eq!(scroll_offset(0, 99, 100, 30), 70);
    }

    #[test]
    fn fills_the_screen_when_the_list_shrinks() {
        // Collapsing groups at the end leaves 50 items.
        assert_eq!(scroll_offset(70, 45, 50, 30), 20);
        assert_eq!(scroll_offset(5, 3, 20, 30), 0);
    }
}
