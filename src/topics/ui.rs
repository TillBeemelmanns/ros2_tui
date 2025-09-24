use ratatui::{
    layout::{Constraint, Direction, Layout},
    style::{Color, Modifier, Style},
    symbols,
    text::{Line, Span},
    widgets::{
        Axis, Block, Borders, Cell, Chart, Dataset, GraphType, Paragraph, Row, Scrollbar,
        ScrollbarOrientation, ScrollbarState, Table, Wrap,
    },
    Frame,
};

use super::app::{App, AppMode, DetailPaneFocus, CHARTS_MAX_DATA_POINTS};
use super::ros;
use super::ros::MeasurementStatus;
use crate::common::{TopicTree, TopicTreeItem};
use std::time::Duration;

pub fn ui(f: &mut Frame, app: &App) {
    match app.mode {
        AppMode::TopicList => render_topic_list(f, app),
        AppMode::TopicDetail => render_topic_detail(f, app),
        AppMode::Search => render_search_mode(f, app),
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
            " Topics Help",
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
        Line::from("    <Enter>     - Toggle watch on topic / expand group"),
        Line::from("    d           - View details for selected topic"),
        Line::from("    c           - Toggle collapse/uncollapse all groups"),
        Line::from("    s           - Toggle sim time for delay metrics"),
        Line::from("    F4          - Enter search/filter mode"),
        Line::from(""),
        Line::from(Span::styled(
            "  General",
            Style::default().add_modifier(Modifier::UNDERLINED),
        )),
        Line::from("    ?           - Show this help screen"),
        Line::from("    q           - Quit the application"),
        Line::from("    <Esc>       - Clear filter, or exit current view, or quit"),
        Line::from(""),
        Line::from(Span::styled(
            "  Press <Esc> or 'q' to close this help window.",
            Style::default().fg(Color::Yellow),
        )),
    ];
    let paragraph = Paragraph::new(help_text).block(create_block("Help"));
    f.render_widget(paragraph, f.area());
}

fn render_topic_list(f: &mut Frame, app: &App) {
    let main_chunks = Layout::default()
        .direction(Direction::Vertical)
        .constraints([Constraint::Min(0), Constraint::Length(3)])
        .split(f.area());

    render_topic_table(f, app, main_chunks[0]);
    render_status_bar(f, app, main_chunks[1]);
}

fn render_status_bar(f: &mut Frame, app: &App, area: ratatui::layout::Rect) {
    let filter_display = if !app.filter_text.is_empty() {
        format!(" | Filter: '{}'", app.filter_text)
    } else {
        String::new()
    };

    let sim_time_span = Span::styled(
        format!("Sim time: {}", if app.use_sim_time { "ON" } else { "OFF" }),
        Style::default().fg(if app.use_sim_time {
            Color::Green
        } else {
            Color::DarkGray
        }),
    );

    let mut spans = if let Some(error) = &app.error_message {
        vec![Span::styled(
            format!("Error: {}", error),
            Style::default().fg(Color::Red),
        )]
    } else if app.mode == AppMode::TopicDetail {
        vec![
            Span::raw("↑↓/jk: Scroll | "),
            Span::styled("PgUp/PgDn", Style::default().add_modifier(Modifier::BOLD)),
            Span::raw(": Page | "),
            Span::styled("Home/End", Style::default().add_modifier(Modifier::BOLD)),
            Span::raw(": Top/Bottom | "),
            Span::styled("Tab", Style::default().add_modifier(Modifier::BOLD)),
            Span::raw(": Switch Pane | "),
            Span::styled("e", Style::default().add_modifier(Modifier::BOLD)),
            Span::raw(": Toggle Echo | "),
            Span::styled("s", Style::default().add_modifier(Modifier::BOLD)),
            Span::raw(": Sim Time | "),
            Span::styled("q/Esc", Style::default().add_modifier(Modifier::BOLD)),
            Span::raw(": Back"),
        ]
    } else {
        let mut base = vec![
            Span::raw("↑↓/jk Navigate | "),
            Span::styled("Enter", Style::default().add_modifier(Modifier::BOLD)),
            Span::raw(": Watch/Expand | "),
            Span::styled("d", Style::default().add_modifier(Modifier::BOLD)),
            Span::raw(": Details | "),
            Span::styled("c", Style::default().add_modifier(Modifier::BOLD)),
            Span::raw(": Un/Collapse | "),
            Span::styled("F4", Style::default().add_modifier(Modifier::BOLD)),
            Span::raw(": Search | "),
            Span::styled("s", Style::default().add_modifier(Modifier::BOLD)),
            Span::raw(": Sim Time | "),
            Span::styled("r", Style::default().add_modifier(Modifier::BOLD)),
            Span::raw(": Refresh | "),
            Span::styled("?", Style::default().add_modifier(Modifier::BOLD)),
            Span::raw(": Help | "),
            Span::styled("q", Style::default().add_modifier(Modifier::BOLD)),
            Span::raw(": Quit"),
        ];
        if !filter_display.is_empty() {
            base.push(Span::styled(
                filter_display,
                Style::default().fg(Color::Yellow),
            ));
        }
        base
    };

    let content_width: usize = spans.iter().map(|span| span.content.len()).sum();
    let indicator_width = sim_time_span.content.len();
    let available_width = area.width.saturating_sub(1) as usize; // Leave room for border rendering quirks
    let padding = available_width
        .saturating_sub(content_width + indicator_width)
        .max(1);
    spans.push(Span::raw(" ".repeat(padding)));
    spans.push(sim_time_span);

    let status = Paragraph::new(Line::from(spans)).block(Block::default().borders(Borders::TOP));
    f.render_widget(status, area);
}

fn render_topic_detail(f: &mut Frame, app: &App) {
    let topic_name = app.selected_topic_name.as_deref().unwrap_or("Unknown");
    let topic_info = app.topic_map.get(topic_name);

    let main_chunks = Layout::default()
        .direction(Direction::Vertical)
        .constraints([Constraint::Min(0), Constraint::Length(3)])
        .split(f.area());

    // Split the main area into top and bottom halves
    let top_bottom_chunks = Layout::default()
        .direction(Direction::Vertical)
        .constraints([Constraint::Percentage(50), Constraint::Percentage(50)])
        .split(main_chunks[0]);

    let top_chunk = top_bottom_chunks[0];
    let echo_pane_chunk = top_bottom_chunks[1];

    // Split the top half into left and right columns
    let top_left_right_chunks = Layout::default()
        .direction(Direction::Horizontal)
        .constraints([Constraint::Percentage(50), Constraint::Percentage(50)])
        .split(top_chunk);

    let detail_info_chunk = top_left_right_chunks[0];
    let chart_chunk = top_left_right_chunks[1];

    // Split the right column into two for the charts
    let chart_chunks = Layout::default()
        .direction(Direction::Vertical)
        .constraints([Constraint::Percentage(50), Constraint::Percentage(50)])
        .split(chart_chunk);

    let hz_chart_chunk = chart_chunks[0];
    let delay_chart_chunk = chart_chunks[1];

    // Render Topic Info Pane
    let mut info_content = vec![];
    if let Some(topic) = topic_info {
        info_content.extend(vec![
            Line::from(vec![
                Span::styled("Topic: ", Style::default().fg(Color::Cyan)),
                Span::raw(&topic.name),
            ]),
            Line::from(vec![
                Span::styled("Message Type: ", Style::default().fg(Color::Cyan)),
                Span::raw(&topic.msg_type),
            ]),
            Line::from(vec![
                Span::styled("Publishers:   ", Style::default().fg(Color::Cyan)),
                Span::raw(topic.publisher_count.to_string()),
            ]),
            Line::from(vec![
                Span::styled("Subscribers:  ", Style::default().fg(Color::Cyan)),
                Span::raw(topic.subscriber_count.to_string()),
            ]),
            Line::from(vec![
                Span::styled("Watched:      ", Style::default().fg(Color::Cyan)),
                Span::raw(if topic.watched { "Yes" } else { "No" }),
            ]),
            Line::from(vec![
                Span::styled("Echo:         ", Style::default().fg(Color::Cyan)),
                Span::raw(if app.is_echoing { "Yes" } else { "No" }),
            ]),
            Line::from(""),
            Line::from(vec![
                Span::styled("Frequency (Hz): ", Style::default().fg(Color::Cyan)),
                Span::raw(match (topic.hz, topic.hz_std_dev) {
                    (Some(hz), Some(std_dev)) if std_dev > 0.0 => {
                        format!("{:.2} ± {:.3}", hz, std_dev)
                    }
                    (Some(hz), _) => format!("{:.2}", hz),
                    _ => "N/A".to_string(),
                }),
            ]),
            Line::from(vec![
                Span::styled("Delay (ms):     ", Style::default().fg(Color::Cyan)),
                Span::raw(match (topic.delay, topic.delay_std_dev) {
                    (Some(delay), Some(std_dev)) if std_dev > 0.0 => {
                        format!("{:.2} ± {:.3}", delay * 1000.0, std_dev * 1000.0)
                    }
                    (Some(delay), _) => format!("{:.2}", delay * 1000.0),
                    _ => "N/A".to_string(),
                }),
            ]),
        ]);
    } else {
        info_content.push(Line::from(Span::raw(format!(
            "Topic '{}' not found.",
            topic_name
        ))));
    }
    let paragraph = Paragraph::new(info_content.clone())
        .block(create_block("Topic Details"))
        .wrap(Wrap { trim: true });
    f.render_widget(paragraph, detail_info_chunk);

    if let Some(topic) = topic_info {
        if topic.watched {
            // Hz Chart with Bollinger Bands - Simple FIFO approach
            // Both histories should ALWAYS have same length since they come from same ROS2 output

            let mut hz_chart_data: Vec<(f64, f64)> = Vec::new();
            let mut upper_band_data: Vec<(f64, f64)> = Vec::new();
            let mut lower_band_data: Vec<(f64, f64)> = Vec::new();

            // Simple approach: both queues should always have identical length
            let data_len = topic.hz_history.len().min(topic.hz_std_dev_history.len());

            for i in 0..data_len {
                let hz_val = topic.hz_history[i];
                let std_dev_val = topic.hz_std_dev_history[i];

                // Main Hz line
                hz_chart_data.push((i as f64, hz_val));

                // Bollinger bands
                let upper_val = hz_val + std_dev_val;
                let lower_val = (hz_val - std_dev_val).max(0.0);
                upper_band_data.push((i as f64, upper_val));
                lower_band_data.push((i as f64, lower_val));
            }

            // Calculate y bounds including Bollinger bands
            let all_values: Vec<f64> = hz_chart_data
                .iter()
                .map(|(_, v)| *v)
                .chain(upper_band_data.iter().map(|(_, v)| *v))
                .chain(lower_band_data.iter().map(|(_, v)| *v))
                .collect();
            let hz_min_val = all_values.iter().cloned().fold(f64::INFINITY, f64::min);
            let hz_max_val = all_values.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
            let hz_y_bounds =
                if hz_min_val.is_finite() && hz_max_val.is_finite() && hz_min_val != hz_max_val {
                    [
                        hz_min_val - (hz_max_val - hz_min_val) * 0.1,
                        hz_max_val + (hz_max_val - hz_min_val) * 0.1,
                    ]
                } else if hz_min_val.is_finite() {
                    [hz_min_val * 0.9, hz_min_val * 1.1]
                } else {
                    [0.0, 1.0]
                };

            let hz_x_bounds = [0.0, CHARTS_MAX_DATA_POINTS as f64];

            let mut datasets = vec![];

            // Simple: always add all three datasets if we have any data
            if data_len > 0 {
                // Upper Bollinger band (Hz + std_dev)
                datasets.push(
                    Dataset::default()
                        .name("Upper Band")
                        .marker(symbols::Marker::Braille)
                        .style(Style::default().fg(Color::Red))
                        .graph_type(GraphType::Line)
                        .data(&upper_band_data),
                );

                // Lower Bollinger band (Hz - std_dev)
                datasets.push(
                    Dataset::default()
                        .name("Lower Band")
                        .marker(symbols::Marker::Braille)
                        .style(Style::default().fg(Color::Red))
                        .graph_type(GraphType::Line)
                        .data(&lower_band_data),
                );

                // Main Hz line (should be drawn on top)
                datasets.push(
                    Dataset::default()
                        .name("Hz")
                        .marker(symbols::Marker::Braille)
                        .style(Style::default().fg(Color::Green))
                        .graph_type(GraphType::Line)
                        .data(&hz_chart_data),
                );
            }

            let hz_chart = Chart::new(datasets)
                .block(create_block("Frequency (Hz)"))
                .x_axis(
                    Axis::default()
                        .title("Time")
                        .style(Style::default().fg(Color::Gray))
                        .bounds(hz_x_bounds)
                        .labels(vec![
                            Span::raw("0"),
                            Span::raw(format!("{}", CHARTS_MAX_DATA_POINTS / 2)),
                            Span::raw(format!("{}", CHARTS_MAX_DATA_POINTS)),
                        ]),
                )
                .y_axis(
                    Axis::default()
                        .title("Hz")
                        .style(Style::default().fg(Color::Gray))
                        .bounds(hz_y_bounds)
                        .labels(vec![
                            Span::raw(format!("{:.1}", hz_y_bounds[0])),
                            Span::raw(format!("{:.1}", hz_y_bounds[1])),
                        ]),
                );
            f.render_widget(hz_chart, hz_chart_chunk);

            // Delay Chart with Bollinger Bands - Simple FIFO approach
            // Both histories should ALWAYS have same length since they come from same ROS2 output

            let mut delay_chart_data: Vec<(f64, f64)> = Vec::new();
            let mut delay_upper_band_data: Vec<(f64, f64)> = Vec::new();
            let mut delay_lower_band_data: Vec<(f64, f64)> = Vec::new();

            // Simple approach: both queues should always have identical length
            let delay_data_len = topic
                .delay_history
                .len()
                .min(topic.delay_std_dev_history.len());

            for i in 0..delay_data_len {
                let delay_val = topic.delay_history[i];
                let std_dev_val = topic.delay_std_dev_history[i];

                // Main delay line
                delay_chart_data.push((i as f64, delay_val));

                // Bollinger bands
                let upper_val = delay_val + std_dev_val;
                let lower_val = (delay_val - std_dev_val).max(0.0);
                delay_upper_band_data.push((i as f64, upper_val));
                delay_lower_band_data.push((i as f64, lower_val));
            }

            // Calculate y bounds including Bollinger bands
            let all_delay_values: Vec<f64> = delay_chart_data
                .iter()
                .map(|(_, v)| *v)
                .chain(delay_upper_band_data.iter().map(|(_, v)| *v))
                .chain(delay_lower_band_data.iter().map(|(_, v)| *v))
                .collect();
            let delay_min_val = all_delay_values
                .iter()
                .cloned()
                .fold(f64::INFINITY, f64::min);
            let delay_max_val = all_delay_values
                .iter()
                .cloned()
                .fold(f64::NEG_INFINITY, f64::max);
            let delay_y_bounds = if delay_min_val.is_finite()
                && delay_max_val.is_finite()
                && delay_min_val != delay_max_val
            {
                [
                    delay_min_val - (delay_max_val - delay_min_val) * 0.1,
                    delay_max_val + (delay_max_val - delay_min_val) * 0.1,
                ]
            } else if delay_min_val.is_finite() {
                [delay_min_val * 0.9, delay_min_val * 1.1]
            } else {
                [0.0, 1.0]
            };

            let delay_x_bounds = [0.0, CHARTS_MAX_DATA_POINTS as f64];

            let mut delay_datasets = vec![];

            // Simple: always add all three datasets if we have any data
            if delay_data_len > 0 {
                // Upper Bollinger band (Delay + std_dev)
                delay_datasets.push(
                    Dataset::default()
                        .name("Upper Band")
                        .marker(symbols::Marker::Braille)
                        .style(Style::default().fg(Color::Red))
                        .graph_type(GraphType::Line)
                        .data(&delay_upper_band_data),
                );

                // Lower Bollinger band (Delay - std_dev)
                delay_datasets.push(
                    Dataset::default()
                        .name("Lower Band")
                        .marker(symbols::Marker::Braille)
                        .style(Style::default().fg(Color::Red))
                        .graph_type(GraphType::Line)
                        .data(&delay_lower_band_data),
                );

                // Main delay line (should be drawn on top)
                delay_datasets.push(
                    Dataset::default()
                        .name("Delay")
                        .marker(symbols::Marker::Braille)
                        .style(Style::default().fg(Color::Green))
                        .graph_type(GraphType::Line)
                        .data(&delay_chart_data),
                );
            }

            let delay_chart = Chart::new(delay_datasets)
                .block(create_block("Delay (ms)"))
                .x_axis(
                    Axis::default()
                        .title("Time")
                        .style(Style::default().fg(Color::Gray))
                        .bounds(delay_x_bounds)
                        .labels(vec![
                            Span::raw("0"),
                            Span::raw(format!("{}", CHARTS_MAX_DATA_POINTS / 2)),
                            Span::raw(format!("{}", CHARTS_MAX_DATA_POINTS)),
                        ]),
                )
                .y_axis(
                    Axis::default()
                        .title("ms")
                        .style(Style::default().fg(Color::Gray))
                        .bounds(delay_y_bounds)
                        .labels(vec![
                            Span::raw(format!("{:.1}", delay_y_bounds[0])),
                            Span::raw(format!("{:.1}", delay_y_bounds[1])),
                        ]),
                );
            f.render_widget(delay_chart, delay_chart_chunk);
        } else {
            // Show helpful text in chart areas when topic is not being watched
            let hz_placeholder = Paragraph::new(vec![
                Line::from(""),
                Line::from(""),
                Line::from(Span::styled(
                    "Press 'Enter' to start watching this topic.",
                    Style::default().fg(Color::Yellow),
                )),
            ])
            .block(create_block("Frequency (Hz)"))
            .alignment(ratatui::layout::Alignment::Center);
            f.render_widget(hz_placeholder, hz_chart_chunk);

            let delay_placeholder = Paragraph::new(vec![
                Line::from(""),
                Line::from(""),
                Line::from(Span::styled(
                    "Press 'Enter' to start watching this topic.",
                    Style::default().fg(Color::Yellow),
                )),
            ])
            .block(create_block("Delay (ms)"))
            .alignment(ratatui::layout::Alignment::Center);
            f.render_widget(delay_placeholder, delay_chart_chunk);
        }
    } else {
        // Show placeholder when topic doesn't exist
        let hz_placeholder = Paragraph::new(vec![
            Line::from(""),
            Line::from(""),
            Line::from(Span::styled(
                "Topic not found",
                Style::default().fg(Color::Red),
            )),
        ])
        .block(create_block("Frequency (Hz)"))
        .alignment(ratatui::layout::Alignment::Center);
        f.render_widget(hz_placeholder, hz_chart_chunk);

        let delay_placeholder = Paragraph::new(vec![
            Line::from(""),
            Line::from(""),
            Line::from(Span::styled(
                "Topic not found",
                Style::default().fg(Color::Red),
            )),
        ])
        .block(create_block("Delay (ms)"))
        .alignment(ratatui::layout::Alignment::Center);
        f.render_widget(delay_placeholder, delay_chart_chunk);
    }

    // Render Echo Pane
    let echo_block = Block::default()
        .title("Echo")
        .borders(Borders::ALL)
        .border_style(if app.detail_focus == DetailPaneFocus::Echo {
            Style::default().fg(Color::Cyan)
        } else {
            Style::default()
        });

    let mut lines: Vec<Line> = app
        .echo_content
        .iter()
        .map(|s| Line::from(s.clone()))
        .collect();
    if !app.is_echoing && app.echo_content.is_empty() {
        lines.push(Line::from(""));
        lines.push(Line::from(Span::styled(
            "Press 'e' to start echoing this topic.",
            Style::default().fg(Color::Yellow),
        )));
    } else if app.is_echoing && app.echo_content.is_empty() {
        lines.push(Line::from(""));
        lines.push(Line::from(Span::styled(
            "Waiting for messages...",
            Style::default().fg(Color::Gray),
        )));
    }
    let paragraph = Paragraph::new(lines)
        .block(echo_block)
        .scroll((app.echo_scroll_offset as u16, 0));
    f.render_widget(paragraph, echo_pane_chunk);

    // Render scrollbar for echo area
    let echo_area_height = echo_pane_chunk.height.saturating_sub(2) as usize; // Account for borders
    let content_length = app.echo_content.len();
    let max_scroll = content_length.saturating_sub(echo_area_height);
    let scroll_position = app.echo_scroll_offset.min(max_scroll);

    let mut scrollbar_state = ScrollbarState::default()
        .content_length(max_scroll.max(1)) // Ensure minimum of 1 to show scrollbar
        .position(scroll_position);

    let scrollbar = Scrollbar::new(ScrollbarOrientation::VerticalRight)
        .begin_symbol(Some("↑"))
        .end_symbol(Some("↓"));
    f.render_stateful_widget(scrollbar, echo_pane_chunk, &mut scrollbar_state);

    render_status_bar(f, app, main_chunks[1]);
}

fn render_topic_table(f: &mut Frame, app: &App, area: ratatui::layout::Rect) {
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

    let visible_items: Vec<&TopicTreeItem> = app
        .visible_items
        .iter()
        .skip(scroll_offset)
        .take(table_height)
        .collect();

    let header_cells = [
        "W",
        "Topic / Group",
        "Type",
        "Pub",
        "Sub",
        "Hz",
        "Delay (ms)",
    ];
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
                    Cell::from(""),
                    Cell::from(""),
                    Cell::from(""),
                ])
                .style(style)
            } else {
                let topic_info = app.topic_map.get(&item.node.full_path).unwrap();
                let watch_indicator = if topic_info.watched { "●" } else { " " };

                let (hz_display, delay_display) = if topic_info.watched {
                    (
                        format_measurement(&topic_info.hz, &topic_info.hz_status, 1.0),
                        format_measurement(&topic_info.delay, &topic_info.delay_status, 1000.0),
                    )
                } else {
                    ("".to_string(), "".to_string())
                };

                let style = if topic_info.watched {
                    Style::default().fg(Color::Green)
                } else {
                    Style::default()
                };

                Row::new(vec![
                    Cell::from(watch_indicator),
                    Cell::from(item.get_display_name()),
                    Cell::from(topic_info.msg_type.clone()),
                    Cell::from(topic_info.publisher_count.to_string()),
                    Cell::from(topic_info.subscriber_count.to_string()),
                    Cell::from(hz_display),
                    Cell::from(delay_display),
                ])
                .style(style)
            }
        })
        .collect();

    let title = "ROS2 Topics";
    let table = Table::new(
        rows,
        &[
            Constraint::Length(2),
            Constraint::Percentage(45),
            Constraint::Percentage(25),
            Constraint::Length(5),
            Constraint::Length(5),
            Constraint::Length(8),
            Constraint::Length(12),
        ],
    )
    .header(header)
    .block(create_block(title))
    .row_highlight_style(Style::default().bg(Color::Blue).fg(Color::White))
    .highlight_symbol(">> ");

    let mut state = ratatui::widgets::TableState::default();
    state.select(Some(app.selected_index.saturating_sub(scroll_offset)));
    f.render_stateful_widget(table, area, &mut state);
}

fn format_measurement(val: &Option<f64>, status: &MeasurementStatus, multiplier: f64) -> String {
    match status {
        MeasurementStatus::NotMeasuring => " ".to_string(),
        MeasurementStatus::Loading(frame) => {
            format!("{}{}", ".".repeat(*frame + 1), " ".repeat(2 - *frame))
        }
        MeasurementStatus::HasValue => {
            val.map_or(" ".to_string(), |v| format!("{:.1}", v * multiplier))
        }
        MeasurementStatus::NoStamp => "-".to_string(),
    }
}

fn render_search_mode(f: &mut Frame, app: &App) {
    let chunks = Layout::default()
        .direction(Direction::Vertical)
        .constraints([Constraint::Min(0), Constraint::Length(3)])
        .split(f.area());

    // Create a temporary app state to render the live search results
    // without modifying the main app's visible_items list.
    let mut temp_app = App::new(Duration::from_secs(1), Duration::from_secs(1));
    temp_app.topic_map = app.topic_map.clone();

    let search_text = &app.search_text;
    let filtered_topics: Vec<ros::TopicInfo> = temp_app
        .topic_map
        .values()
        .filter(|topic| {
            topic
                .name
                .to_lowercase()
                .contains(&search_text.to_lowercase())
        })
        .cloned()
        .collect();

    let mut filtered_tree = TopicTree::new();
    // Convert ros::TopicInfo to tree::TopicInfo
    let tree_topics: Vec<crate::tree::TopicInfo> = filtered_topics
        .iter()
        .map(|t| crate::tree::TopicInfo {
            name: t.name.clone(),
        })
        .collect();
    filtered_tree.build_from_topics(&tree_topics);

    // Automatically expand all groups in the live search preview
    for group in filtered_tree.root.values_mut() {
        group.is_expanded = true;
    }

    temp_app.visible_items = filtered_tree.get_flattened_view();
    temp_app.selected_index = app.selected_index;
    if temp_app.selected_index >= temp_app.visible_items.len() {
        temp_app.selected_index = temp_app.visible_items.len().saturating_sub(1);
    }

    render_topic_table(f, &temp_app, chunks[0]);

    let search_prompt = Paragraph::new(Line::from(vec![
        Span::styled("Search: ", Style::default().fg(Color::Yellow)),
        Span::raw(&app.search_text),
        Span::styled(
            "█",
            Style::default()
                .fg(Color::White)
                .add_modifier(Modifier::SLOW_BLINK),
        ),
    ]))
    .block(
        Block::default()
            .borders(Borders::TOP)
            .title("[Search Mode - Enter to confirm, Esc to cancel]"),
    );

    f.render_widget(search_prompt, chunks[1]);
}
