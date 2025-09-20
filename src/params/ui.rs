use ratatui::{
    layout::{Constraint, Direction, Layout},
    style::{Color, Modifier, Style},
    text::{Line, Span},
    widgets::{Block, Borders, Cell, Paragraph, Row, Table, TableState, Wrap},
    Frame,
};

use super::app::{AppMode, ParamsApp};
use crate::common::{ParamTreeItem, PopupField, PopupType, UniformPopup};

pub fn ui(f: &mut Frame, app: &ParamsApp) {
    match app.mode {
        AppMode::ParamList => render_param_list(f, app),
        AppMode::ParamDetail => render_param_detail(f, app),
        AppMode::Search => render_search_mode(f, app),
        AppMode::SetParameter => render_set_parameter_dialog(f, app),
        AppMode::DumpParameters => render_dump_parameters_dialog(f, app),
        AppMode::LoadParameters => render_load_parameters_dialog(f, app),
        AppMode::Warning => render_warning_dialog(f, app),
        AppMode::Help => render_help(f),
    }
}

fn create_block(title: &str) -> Block<'_> {
    Block::default().borders(Borders::ALL).title(Span::styled(
        title,
        Style::default().add_modifier(Modifier::BOLD),
    ))
}

fn render_help(f: &mut Frame) {
    let help_text = vec![
        Line::from(""),
        Line::from(Span::styled(
            " Parameters Help",
            Style::default()
                .add_modifier(Modifier::BOLD)
                .fg(Color::Cyan),
        )),
        Line::from(""),
        Line::from(Span::styled(
            "  Navigation",
            Style::default().add_modifier(Modifier::UNDERLINED),
        )),
        Line::from("    ↑ / k       - Move selection up"),
        Line::from("    ↓ / j       - Move selection down"),
        Line::from("    ← / h       - Collapse selected group"),
        Line::from("    → / l       - Expand selected group"),
        Line::from(""),
        Line::from(Span::styled(
            "  Actions",
            Style::default().add_modifier(Modifier::UNDERLINED),
        )),
        Line::from("    <Enter>     - Expand/collapse group"),
        Line::from("    s           - Set parameter value"),
        Line::from("    d           - Dump parameters to YAML file"),
        Line::from("    Ctrl+l      - Load parameters from YAML file"),
        Line::from("    c           - Toggle collapse/uncollapse all groups"),
        Line::from("    F4          - Enter search/filter mode"),
        Line::from(""),
        Line::from(Span::styled(
            "  General",
            Style::default().add_modifier(Modifier::UNDERLINED),
        )),
        Line::from("    ?           - Show this help screen"),
        Line::from("    r / F5      - Refresh parameter list"),
        Line::from("    Space       - Refresh parameter list"),
        Line::from("    q / Ctrl+C  - Quit"),
        Line::from("    Esc         - Quit / Cancel current operation"),
        Line::from(""),
        Line::from(Span::styled(
            "  Legend",
            Style::default().add_modifier(Modifier::UNDERLINED),
        )),
        Line::from("    ●          - Parameter is being watched"),
        Line::from("    ...        - Loading parameter value"),
        Line::from("    -          - Parameter read-only or no value"),
        Line::from(""),
        Line::from("Press Esc to return to parameter list"),
    ];

    let paragraph = Paragraph::new(help_text)
        .block(create_block("Help"))
        .wrap(Wrap { trim: false });

    f.render_widget(paragraph, f.area());
}

fn render_search_mode(f: &mut Frame, app: &ParamsApp) {
    let chunks = Layout::default()
        .direction(Direction::Vertical)
        .constraints([Constraint::Min(0), Constraint::Length(3)])
        .split(f.area());

    render_param_table(f, app, chunks[0]);

    let search_input = Paragraph::new(format!("Search: {}", app.search_text))
        .block(create_block(
            "Search Mode - Type to filter, Enter to apply, Esc to cancel",
        ))
        .style(Style::default().fg(Color::Yellow));

    f.render_widget(search_input, chunks[1]);
}

fn render_param_list(f: &mut Frame, app: &ParamsApp) {
    let main_chunks = if app.error_message.is_some() || app.success_message.is_some() {
        Layout::default()
            .direction(Direction::Vertical)
            .constraints([
                Constraint::Min(0),
                Constraint::Length(3),
                Constraint::Length(3),
            ])
            .split(f.area())
    } else {
        Layout::default()
            .direction(Direction::Vertical)
            .constraints([Constraint::Min(0), Constraint::Length(3)])
            .split(f.area())
    };

    render_param_table(f, app, main_chunks[0]);

    // Render error/success message if present
    if main_chunks.len() > 2 {
        if let Some(error) = &app.error_message {
            let error_paragraph = Paragraph::new(error.as_str())
                .block(create_block("Error"))
                .style(Style::default().fg(Color::Red))
                .wrap(Wrap { trim: false });
            f.render_widget(error_paragraph, main_chunks[1]);
        } else if let Some(success) = &app.success_message {
            let success_paragraph = Paragraph::new(success.as_str())
                .block(create_block("Success"))
                .style(Style::default().fg(Color::Green))
                .wrap(Wrap { trim: false });
            f.render_widget(success_paragraph, main_chunks[1]);
        }
        render_status_bar(f, app, main_chunks[2]);
    } else {
        render_status_bar(f, app, main_chunks[1]);
    }
}

fn render_param_table(f: &mut Frame, app: &ParamsApp, area: ratatui::layout::Rect) {
    let table_height = area.height.saturating_sub(2) as usize;
    let mut scroll_offset = app.scroll_offset;
    if app.selected_index >= scroll_offset.saturating_add(table_height) {
        scroll_offset = app
            .selected_index
            .saturating_sub(table_height)
            .saturating_add(1);
    }
    if app.selected_index < scroll_offset {
        scroll_offset = app.selected_index;
    }

    let visible_items: Vec<&ParamTreeItem> = app
        .visible_items
        .iter()
        .skip(scroll_offset)
        .take(table_height)
        .collect();

    let header_cells = ["W", "Parameter / Node", "Type", "Value"];
    let header = Row::new(header_cells).style(
        Style::default()
            .fg(Color::Cyan)
            .add_modifier(Modifier::BOLD),
    );

    let rows: Vec<Row> = visible_items
        .iter()
        .map(|item| {
            if item.is_group() {
                let style = Style::default()
                    .fg(Color::Yellow)
                    .add_modifier(Modifier::BOLD);
                Row::new(vec![
                    Cell::from(""),
                    Cell::from(item.get_display_name()),
                    Cell::from(""),
                    Cell::from(""),
                ])
                .style(style)
            } else {
                let key = &item.node.full_path;
                if let Some(param) = app.param_map.get(key) {
                    let param_display_name = format!("  {}", item.get_display_name());
                    let param_type = if param.param_type.is_empty() {
                        "-".to_string()
                    } else {
                        param.param_type.clone()
                    };
                    let param_value = param.value.clone().unwrap_or_else(|| "-".to_string());

                    Row::new(vec![
                        Cell::from(""), // No watch indicator needed anymore
                        Cell::from(param_display_name),
                        Cell::from(param_type),
                        Cell::from(param_value),
                    ])
                } else {
                    Row::new(vec![
                        Cell::from(""),
                        Cell::from(""),
                        Cell::from(""),
                        Cell::from(""),
                    ])
                }
            }
        })
        .collect();

    let title = if !app.filter_text.is_empty() {
        format!("ROS2 Parameters (filtered: '{}')", app.filter_text)
    } else {
        "ROS2 Parameters".to_string()
    };

    let table = Table::new(
        rows,
        &[
            Constraint::Length(2),
            Constraint::Percentage(45),
            Constraint::Percentage(15),
            Constraint::Percentage(38),
        ],
    )
    .header(header)
    .block(create_block(&title))
    .row_highlight_style(Style::default().bg(Color::Blue).fg(Color::White))
    .highlight_symbol(">> ");

    let mut state = TableState::default();
    state.select(Some(app.selected_index.saturating_sub(scroll_offset)));
    f.render_stateful_widget(table, area, &mut state);
}

fn render_status_bar(f: &mut Frame, app: &ParamsApp, area: ratatui::layout::Rect) {
    let filter_display = if !app.filter_text.is_empty() {
        format!(" | Filter: '{}'", app.filter_text)
    } else {
        String::new()
    };

    let status_text = match app.mode {
        AppMode::ParamList => Line::from(vec![
            Span::raw("↑↓/jk Navigate | "),
            Span::styled("Enter", Style::default().add_modifier(Modifier::BOLD)),
            Span::raw(": Expand | "),
            Span::styled("s", Style::default().add_modifier(Modifier::BOLD)),
            Span::raw(": Set | "),
            Span::styled("c", Style::default().add_modifier(Modifier::BOLD)),
            Span::raw(": Un/Collapse | "),
            Span::styled("d", Style::default().add_modifier(Modifier::BOLD)),
            Span::raw(": Dump | "),
            Span::styled("Ctrl+l", Style::default().add_modifier(Modifier::BOLD)),
            Span::raw(": Load | "),
            Span::styled("F4", Style::default().add_modifier(Modifier::BOLD)),
            Span::raw(": Search | "),
            Span::styled("r", Style::default().add_modifier(Modifier::BOLD)),
            Span::raw(": Refresh | "),
            Span::styled("?", Style::default().add_modifier(Modifier::BOLD)),
            Span::raw(": Help | "),
            Span::styled("q", Style::default().add_modifier(Modifier::BOLD)),
            Span::raw(": Quit"),
            Span::styled(filter_display, Style::default().fg(Color::Yellow)),
        ]),
        AppMode::Search => {
            Line::from("Search Mode - Type to filter parameters, Enter to apply, Esc to cancel")
        }
        _ => Line::from(""),
    };

    let status = Paragraph::new(status_text).block(Block::default().borders(Borders::TOP));
    f.render_widget(status, area);
}

fn render_param_detail(f: &mut Frame, app: &ParamsApp) {
    if let Some(param_key) = &app.selected_param_key {
        if let Some(param) = app.param_map.get(param_key) {
            let main_chunks = Layout::default()
                .direction(Direction::Vertical)
                .constraints([Constraint::Min(0), Constraint::Length(3)])
                .split(f.area());

            let chunks = Layout::default()
                .direction(Direction::Vertical)
                .constraints([
                    Constraint::Percentage(60), // Parameter info
                    Constraint::Percentage(40), // Parameter actions
                ])
                .split(main_chunks[0]);

            // Parameter information
            let info_lines = vec![
                Line::from(vec![
                    Span::styled("Node: ", Style::default().add_modifier(Modifier::BOLD)),
                    Span::from(param.node_name.clone()),
                ]),
                Line::from(vec![
                    Span::styled("Parameter: ", Style::default().add_modifier(Modifier::BOLD)),
                    Span::from(param.param_name.clone()),
                ]),
                Line::from(vec![
                    Span::styled("Type: ", Style::default().add_modifier(Modifier::BOLD)),
                    Span::from(param.param_type.clone()),
                ]),
                Line::from(vec![
                    Span::styled("Value: ", Style::default().add_modifier(Modifier::BOLD)),
                    Span::from(
                        param
                            .value
                            .clone()
                            .unwrap_or_else(|| "Not loaded".to_string()),
                    ),
                ]),
            ];

            let info_paragraph = Paragraph::new(info_lines)
                .block(create_block("Parameter Information"))
                .wrap(Wrap { trim: false });

            f.render_widget(info_paragraph, chunks[0]);

            // Actions section
            let actions_lines = vec![
                Line::from(""),
                Line::from("Available Actions:"),
                Line::from("  s - Set new value"),
                Line::from("  d - Dump all parameters for this node"),
                Line::from("  Ctrl+l - Load parameters for this node"),
                Line::from("  Space - Go back to parameter list"),
                Line::from("  Esc - Go back to parameter list"),
            ];

            let actions_paragraph = Paragraph::new(actions_lines)
                .block(create_block("Actions"))
                .wrap(Wrap { trim: false });

            f.render_widget(actions_paragraph, chunks[1]);

            render_status_bar(f, app, main_chunks[1]);
        }
    }
}

fn render_set_parameter_dialog(f: &mut Frame, app: &ParamsApp) {
    // First render the background
    render_param_list(f, app);

    if let Some(edit_state) = &app.edit_state {
        let popup = UniformPopup::new("Set Parameter Value", PopupType::Input)
            .with_size(60, 40)
            .add_field(PopupField::new("Node", &edit_state.node_name))
            .add_field(PopupField::new("Parameter", &edit_state.param_name))
            .add_field(PopupField::new("Type", &edit_state.param_type))
            .add_field(PopupField::new("Current Value", &edit_state.current_value))
            .add_field(
                PopupField::new("New Value", &edit_state.new_value).editable(app.edit_cursor),
            )
            .with_footer("Enter - Confirm | Esc - Cancel | ←→ - Move cursor");

        popup.render(f, f.area());
    }
}

fn render_dump_parameters_dialog(f: &mut Frame, app: &ParamsApp) {
    // First render the background
    render_param_list(f, app);

    if let Some(item) = app.visible_items.get(app.selected_index) {
        if item.is_group() {
            let popup = UniformPopup::new("Dump Parameters to File", PopupType::Input)
                .add_field(PopupField::new("Node", &item.node.name))
                .add_field(PopupField::new("File Path", &app.file_input).editable(app.file_cursor))
                .with_footer("Enter - Confirm | Esc - Cancel | ←→ - Move cursor");

            popup.render(f, f.area());
        }
    }
}

fn render_load_parameters_dialog(f: &mut Frame, app: &ParamsApp) {
    // First render the background
    render_param_list(f, app);

    if let Some(item) = app.visible_items.get(app.selected_index) {
        if item.is_group() {
            let popup = UniformPopup::new("Load Parameters from File", PopupType::Input)
                .add_field(PopupField::new("Node", &item.node.name))
                .add_field(PopupField::new("File Path", &app.file_input).editable(app.file_cursor))
                .with_footer("Enter - Confirm | Esc - Cancel | ←→ - Move cursor");

            popup.render(f, f.area());
        }
    }
}

fn render_warning_dialog(f: &mut Frame, app: &ParamsApp) {
    // First render the background
    render_param_list(f, app);

    // Check if this is a type validation error to show as error instead of warning
    let (title, popup_type) = if app
        .warning_message
        .contains("Invalid value for parameter type")
    {
        ("❌ Type Validation Error", PopupType::Error)
    } else {
        ("⚠ Warning", PopupType::Warning)
    };

    let popup = UniformPopup::new(title, popup_type)
        .with_size(70, 25)
        .add_field(PopupField::new("", &app.warning_message))
        .with_footer("Press ESC to return to parameter editing or any other key to continue...");

    popup.render(f, f.area());
}
