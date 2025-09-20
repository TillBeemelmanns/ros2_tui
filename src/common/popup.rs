use ratatui::{
    layout::{Position, Rect},
    style::{Color, Modifier, Style},
    text::{Line, Span},
    widgets::{Block, Borders, Clear, Paragraph, Wrap},
    Frame,
};

pub enum PopupType {
    Info,
    Warning,
    Error,
    Input,
}

pub struct PopupField {
    pub label: String,
    pub value: String,
    pub is_editable: bool,
    pub cursor_position: Option<usize>,
    pub style: Style,
}

impl PopupField {
    pub fn new(label: &str, value: &str) -> Self {
        Self {
            label: label.to_string(),
            value: value.to_string(),
            is_editable: false,
            cursor_position: None,
            style: Style::default(),
        }
    }

    pub fn editable(mut self, cursor_pos: usize) -> Self {
        self.is_editable = true;
        self.cursor_position = Some(cursor_pos);
        self.style = Style::default()
            .fg(Color::Yellow)
            .add_modifier(Modifier::UNDERLINED);
        self
    }
}

pub struct UniformPopup {
    pub title: String,
    pub popup_type: PopupType,
    pub fields: Vec<PopupField>,
    pub footer_text: Option<String>,
    pub width_percent: u16,
    pub height_percent: u16,
}

impl UniformPopup {
    pub fn new(title: &str, popup_type: PopupType) -> Self {
        Self {
            title: title.to_string(),
            popup_type,
            fields: Vec::new(),
            footer_text: None,
            width_percent: 60,
            height_percent: 30,
        }
    }

    pub fn add_field(mut self, field: PopupField) -> Self {
        self.fields.push(field);
        self
    }

    pub fn with_footer(mut self, footer: &str) -> Self {
        self.footer_text = Some(footer.to_string());
        self
    }

    pub fn with_size(mut self, width_percent: u16, height_percent: u16) -> Self {
        self.width_percent = width_percent;
        self.height_percent = height_percent;
        self
    }

    pub fn render(&self, f: &mut Frame, background_area: Rect) {
        // Create a centered dialog box
        let popup_area = centered_rect(self.width_percent, self.height_percent, background_area);

        f.render_widget(Clear, popup_area);

        let mut dialog_lines = vec![Line::from("")]; // Empty line at the top

        // Add fields
        for field in &self.fields {
            if field.label.is_empty() {
                // Just the value (for simple text or warnings)
                dialog_lines.push(Line::from(Span::styled(&field.value, field.style)));
            } else {
                // Label: Value format
                dialog_lines.push(Line::from(vec![
                    Span::styled(
                        format!("{}: ", field.label),
                        Style::default().add_modifier(Modifier::BOLD),
                    ),
                    Span::styled(&field.value, field.style),
                ]));
            }
        }

        dialog_lines.push(Line::from(""));

        // Add footer if present
        if let Some(footer) = &self.footer_text {
            dialog_lines.push(Line::from(Span::styled(
                footer,
                Style::default()
                    .add_modifier(Modifier::ITALIC)
                    .fg(Color::Gray),
            )));
        }

        let title_style = match self.popup_type {
            PopupType::Warning => Style::default()
                .add_modifier(Modifier::BOLD)
                .fg(Color::Yellow),
            PopupType::Error => Style::default().add_modifier(Modifier::BOLD).fg(Color::Red),
            PopupType::Info => Style::default().add_modifier(Modifier::BOLD),
            PopupType::Input => Style::default()
                .add_modifier(Modifier::BOLD)
                .fg(Color::Cyan),
        };

        let background_style = match self.popup_type {
            PopupType::Warning | PopupType::Error => Style::default().bg(Color::DarkGray),
            PopupType::Info | PopupType::Input => Style::default().bg(Color::DarkGray),
        };

        let dialog = Paragraph::new(dialog_lines)
            .block(
                Block::default()
                    .borders(Borders::ALL)
                    .title(Span::styled(&self.title, title_style)),
            )
            .wrap(Wrap { trim: false })
            .style(background_style);

        f.render_widget(dialog, popup_area);

        // Set cursor position if there's an editable field
        for (field_index, field) in self.fields.iter().enumerate() {
            if let Some(cursor_pos) = field.cursor_position {
                let cursor_x = popup_area.x + field.label.len() as u16 + 2 + cursor_pos as u16 + 1; // +2 for ": ", +1 for border
                let cursor_y = popup_area.y + 1 + field_index as u16 + 1; // +1 for empty line at top, +1 for border
                f.set_cursor_position(Position::new(cursor_x, cursor_y));
                break; // Only one cursor can be active
            }
        }
    }
}

fn centered_rect(percent_x: u16, percent_y: u16, r: Rect) -> Rect {
    use ratatui::layout::{Constraint, Direction, Layout};

    let popup_layout = Layout::default()
        .direction(Direction::Vertical)
        .constraints([
            Constraint::Percentage((100 - percent_y) / 2),
            Constraint::Percentage(percent_y),
            Constraint::Percentage((100 - percent_y) / 2),
        ])
        .split(r);

    Layout::default()
        .direction(Direction::Horizontal)
        .constraints([
            Constraint::Percentage((100 - percent_x) / 2),
            Constraint::Percentage(percent_x),
            Constraint::Percentage((100 - percent_x) / 2),
        ])
        .split(popup_layout[1])[1]
}
