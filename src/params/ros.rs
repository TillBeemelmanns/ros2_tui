use serde::{Deserialize, Serialize};
use serde_json::Value;

#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct ParamInfo {
    pub node_name: String,
    pub param_name: String,
    pub value: Option<String>,
    pub param_type: String,
}

/// Name of an `rcl_interfaces/msg/ParameterType` constant.
pub fn type_name(param_type: u8) -> &'static str {
    match param_type {
        1 => "Boolean",
        2 => "Integer",
        3 => "Double",
        4 => "String",
        5 => "Byte[]",
        6 => "Boolean[]",
        7 => "Integer[]",
        8 => "Double[]",
        9 => "String[]",
        _ => "Not set",
    }
}

/// Display text of a parameter value of `param_type`, as the helper sends
/// it: a JSON value, with non-finite doubles as text.
pub fn format_value(param_type: u8, value: &Value) -> String {
    match value {
        Value::Array(items) => format_array(items, param_type == 9),
        other => format_scalar(other, false),
    }
}

fn format_scalar(value: &Value, quote_strings: bool) -> String {
    match value {
        Value::Bool(b) => b.to_string(),
        // Doubles always show a decimal point, so they read as doubles.
        Value::Number(n) if n.is_f64() => {
            let text = n.as_f64().unwrap_or_default().to_string();
            if text.contains(['.', 'e', 'E']) {
                text
            } else {
                format!("{}.0", text)
            }
        }
        Value::Number(n) => n.to_string(),
        Value::String(s) if quote_strings => format!("\"{}\"", s),
        Value::String(s) => s.clone(),
        other => other.to_string(),
    }
}

/// An array, shortened with `...` when long.
fn format_array(items: &[Value], quote_strings: bool) -> String {
    const MAX_DISPLAY_ITEMS: usize = 8;
    const MAX_TOTAL_LENGTH: usize = 100;

    let mut elements = Vec::new();
    let mut total_length = 2; // the brackets
    let mut truncated = false;
    for (i, item) in items.iter().enumerate() {
        let text = format_scalar(item, quote_strings);
        let separator_len = if i == 0 { 0 } else { 2 };
        if i >= MAX_DISPLAY_ITEMS || total_length + separator_len + text.len() > MAX_TOTAL_LENGTH {
            truncated = true;
            break;
        }
        total_length += separator_len + text.len();
        elements.push(text);
    }
    if truncated {
        elements.push("...".to_string());
    }
    format!("[{}]", elements.join(", "))
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    #[test]
    fn formats_values_by_type() {
        assert_eq!(format_value(1, &json!(true)), "true");
        assert_eq!(format_value(2, &json!(42)), "42");
        assert_eq!(format_value(3, &json!(1.0)), "1.0");
        assert_eq!(format_value(3, &json!(0.25)), "0.25");
        assert_eq!(format_value(3, &json!("nan")), "nan");
        assert_eq!(format_value(4, &json!("map")), "map");
        assert_eq!(format_value(7, &json!([1, 2, 3])), "[1, 2, 3]");
        assert_eq!(format_value(8, &json!([1.0, 2.5])), "[1.0, 2.5]");
        assert_eq!(format_value(9, &json!(["a", "b"])), "[\"a\", \"b\"]");
        assert_eq!(format_value(6, &json!([])), "[]");
        assert_eq!(type_name(8), "Double[]");
    }

    #[test]
    fn shortens_long_arrays() {
        let long: Vec<i64> = (0..20).collect();
        assert_eq!(
            format_value(7, &json!(long)),
            "[0, 1, 2, 3, 4, 5, 6, 7, ...]"
        );
    }
}
