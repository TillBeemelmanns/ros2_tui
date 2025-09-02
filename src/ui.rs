use ratatui::{
    layout::{Constraint, Direction, Layout, Alignment},
    style::{Color, Style, Modifier},
    text::{Line, Span},
    widgets::{Block, Borders, Paragraph, Row, Table, Wrap, Cell},
    Frame,
};

use crate::app::App;
use crate::ros::MeasurementStatus;

pub fn ui(f: &mut Frame, app: &App) {
    let main_chunks = Layout::default()
        .direction(Direction::Vertical)
        .constraints([
            Constraint::Percentage(70), 
            Constraint::Percentage(25),
            Constraint::Min(3),
        ])
        .split(f.area());

    render_topic_table(f, main_chunks[0], app);
    render_topic_details(f, main_chunks[1], app);
    render_status_bar(f, main_chunks[2], app);
}

fn render_topic_table(f: &mut Frame, area: ratatui::layout::Rect, app: &App) {
    let header_cells = vec![
        "W", // Watch indicator
        "Topic Name",
        "Message Type", 
        "Pub",
        "Sub",
        "Hz",
        "Delay (ms)",
    ];
    
    let header = Row::new(header_cells)
        .style(Style::default()
            .fg(Color::Cyan)
            .add_modifier(Modifier::BOLD));

    // Calculate available table height (minus header and borders)
    let table_height = area.height.saturating_sub(3) as usize; // 1 header + 2 borders
    let (scroll_offset, visible_topics) = app.get_visible_topics(table_height);

    let rows: Vec<Row> = visible_topics.iter().enumerate().map(|(display_idx, topic)| {
        let actual_idx = scroll_offset + display_idx;
        let hz_display = if topic.watched {
            match &topic.hz_status {
                MeasurementStatus::NotMeasuring => "N/A".to_string(),
                MeasurementStatus::Loading(frame) => {
                    let dots = match frame {
                        0 => "   ",
                        1 => ".  ",
                        2 => ".. ",
                        _ => "...",
                    };
                    dots.to_string()
                }
                MeasurementStatus::HasValue => {
                    if let Some(h) = topic.hz {
                        format!("{:.1}", h)
                    } else {
                        // If we have HasValue status but no value, show dots while waiting
                        "...".to_string()
                    }
                }
                MeasurementStatus::NoStamp => "-".to_string(),
            }
        } else {
            "".to_string()
        };
        
        let delay_display = if topic.watched {
            match &topic.delay_status {
                MeasurementStatus::NotMeasuring => "N/A".to_string(),
                MeasurementStatus::Loading(frame) => {
                    let dots = match frame {
                        0 => "   ",
                        1 => ".  ",
                        2 => ".. ",
                        _ => "...",
                    };
                    dots.to_string()
                }
                MeasurementStatus::HasValue => {
                    if let Some(d) = topic.delay {
                        format!("{:.1}", d * 1000.0)
                    } else {
                        // If we have HasValue status but no value, show dots while waiting
                        "...".to_string()
                    }
                }
                MeasurementStatus::NoStamp => "No Stamp".to_string(),
            }
        } else {
            "".to_string()
        };
        
        let watch_indicator = if topic.watched { "●" } else { " " };
        
        let cells = vec![
            watch_indicator.to_string(),
            topic.name.clone(),
            topic.msg_type.clone(),
            topic.publisher_count.to_string(),
            topic.subscriber_count.to_string(),
            hz_display.clone(),
            delay_display.clone(),
        ];
        
        let mut style = Style::default();
        
        // Highlight selected row
        if Some(actual_idx) == app.selected_topic_index {
            style = style.bg(Color::Blue).fg(Color::White);
        }
        // Highlight watched topics
        else if topic.watched {
            style = style.fg(Color::Green).add_modifier(Modifier::BOLD);
        }
        
        // Create row with special styling for loading indicators
        let mut row = Row::new(cells).style(style);
        
        // Apply yellow color to loading dots if they contain animation
        if topic.watched && (matches!(&topic.hz_status, MeasurementStatus::Loading(_)) || 
                           matches!(&topic.delay_status, MeasurementStatus::Loading(_))) {
            // We need to style individual cells for loading animation
            let hz_cell = if matches!(&topic.hz_status, MeasurementStatus::Loading(_)) {
                Cell::from(hz_display).style(Style::default().fg(Color::Yellow).add_modifier(Modifier::BOLD))
            } else {
                Cell::from(hz_display)
            };
            
            let delay_cell = if matches!(&topic.delay_status, MeasurementStatus::Loading(_)) {
                Cell::from(delay_display).style(Style::default().fg(Color::Yellow).add_modifier(Modifier::BOLD))
            } else {
                Cell::from(delay_display)
            };
            
            let styled_cells = vec![
                Cell::from(watch_indicator),
                Cell::from(topic.name.clone()),
                Cell::from(topic.msg_type.clone()),
                Cell::from(topic.publisher_count.to_string()),
                Cell::from(topic.subscriber_count.to_string()),
                hz_cell,
                delay_cell,
            ];
            
            row = Row::new(styled_cells).style(style);
        }
        
        row
    }).collect();

    let total_topics = app.topics.len();
    let title = if total_topics > table_height {
        format!("ROS2 Topics Monitor ({}/{} topics)", scroll_offset + 1, total_topics)
    } else {
        format!("ROS2 Topics Monitor ({} topics)", total_topics)
    };

    let table = Table::new(rows, &[
            Constraint::Length(2),  // Watch indicator
            Constraint::Percentage(45),  // Topic name (increased)
            Constraint::Percentage(25),  // Message type 
            Constraint::Percentage(5),   // Pub
            Constraint::Percentage(5),   // Sub  
            Constraint::Percentage(8),   // Hz (reduced)
            Constraint::Percentage(10),  // Delay (reduced)
        ])
        .header(header)
        .block(Block::default()
            .borders(Borders::ALL)
            .title(title)
            .title_alignment(Alignment::Center));

    f.render_widget(table, area);
}

fn render_topic_details(f: &mut Frame, area: ratatui::layout::Rect, app: &App) {
    let detail_text = if let Some(index) = app.selected_topic_index {
        if let Some(topic) = app.topics.get(index) {
            let mut lines = vec![
                Line::from(vec![
                    Span::styled("Topic: ", Style::default().fg(Color::Green).add_modifier(Modifier::BOLD)),
                    Span::raw(&topic.name),
                ]),
                Line::from(vec![
                    Span::styled("Type: ", Style::default().fg(Color::Green).add_modifier(Modifier::BOLD)),
                    Span::raw(&topic.msg_type),
                ]),
                Line::from(vec![
                    Span::styled("Publishers: ", Style::default().fg(Color::Green).add_modifier(Modifier::BOLD)),
                    Span::raw(topic.publisher_count.to_string()),
                ]),
                Line::from(vec![
                    Span::styled("Subscribers: ", Style::default().fg(Color::Green).add_modifier(Modifier::BOLD)),
                    Span::raw(topic.subscriber_count.to_string()),
                ]),
            ];

            if let Some(hz) = topic.hz {
                lines.push(Line::from(vec![
                    Span::styled("Frequency: ", Style::default().fg(Color::Green).add_modifier(Modifier::BOLD)),
                    Span::raw(format!("{:.2} Hz", hz)),
                ]));
            } else {
                lines.push(Line::from(vec![
                    Span::styled("Frequency: ", Style::default().fg(Color::Green).add_modifier(Modifier::BOLD)),
                    Span::styled("N/A", Style::default().fg(Color::Red)),
                ]));
            }

            if let Some(delay) = topic.delay {
                lines.push(Line::from(vec![
                    Span::styled("Delay: ", Style::default().fg(Color::Green).add_modifier(Modifier::BOLD)),
                    Span::raw(format!("{:.2} ms", delay * 1000.0)),
                ]));
            } else {
                lines.push(Line::from(vec![
                    Span::styled("Delay: ", Style::default().fg(Color::Green).add_modifier(Modifier::BOLD)),
                    Span::styled("N/A", Style::default().fg(Color::Red)),
                ]));
            }


            lines.push(Line::from(""));
            lines.push(Line::from(vec![
                Span::styled("Controls: ", Style::default().fg(Color::Yellow).add_modifier(Modifier::BOLD)),
            ]));
            lines.push(Line::from("↑/↓: Navigate  Enter/Space: Toggle watch  q: Quit  r: Refresh"));

            lines
        } else {
            vec![Line::from("No topic selected")]
        }
    } else {
        vec![Line::from("No topics available")]
    };

    let details = Paragraph::new(detail_text)
        .block(Block::default()
            .borders(Borders::ALL)
            .title("Topic Details")
            .title_alignment(Alignment::Center))
        .wrap(Wrap { trim: true });
    
    f.render_widget(details, area);
}

fn render_status_bar(f: &mut Frame, area: ratatui::layout::Rect, app: &App) {
    let status_text = if let Some(ref error) = app.error_message {
        vec![Line::from(vec![
            Span::styled("Error: ", Style::default().fg(Color::Red).add_modifier(Modifier::BOLD)),
            Span::styled(error, Style::default().fg(Color::Red)),
        ])]
    } else {
        let topic_count = app.topics.len();
        let status_msg = if topic_count == 0 {
            "No topics found - waiting for ROS2 topics...".to_string()
        } else {
            format!("Monitoring {} topics", topic_count)
        };
        
        vec![
            Line::from(vec![
                Span::styled("Status: ", Style::default().fg(Color::Green).add_modifier(Modifier::BOLD)),
                Span::raw(status_msg),
                Span::raw(" | "),
                Span::styled("toptop v0.1.0", Style::default().fg(Color::Blue)),
            ]),
            Line::from("Press 'q' to quit, ↑/↓ to navigate, Enter/Space to watch, 'r' to refresh")
        ]
    };

    let status = Paragraph::new(status_text)
        .block(Block::default()
            .borders(Borders::ALL)
            .title("Status")
            .title_alignment(Alignment::Center));
    
    f.render_widget(status, area);
}