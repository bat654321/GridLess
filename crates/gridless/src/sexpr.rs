//! Minimal S-expression parser for KiCad files. Every list remembers its byte span in
//! the source so that a file can be edited surgically (remove / append nodes) while
//! leaving everything else byte-identical.

use anyhow::{bail, Result};

#[derive(Clone, Debug)]
pub enum Sx {
    Atom(String),
    Str(String),
    List(SList),
}

#[derive(Clone, Debug)]
pub struct SList {
    pub items: Vec<Sx>,
    /// byte offsets of '(' and one past ')'
    pub span: (usize, usize),
}

impl Sx {
    pub fn as_list(&self) -> Option<&SList> {
        match self {
            Sx::List(l) => Some(l),
            _ => None,
        }
    }
    /// Atom or string contents.
    pub fn text(&self) -> Option<&str> {
        match self {
            Sx::Atom(s) | Sx::Str(s) => Some(s),
            _ => None,
        }
    }
    pub fn num(&self) -> Option<f64> {
        self.text().and_then(|s| s.parse().ok())
    }
}

impl SList {
    /// Head symbol of the list, e.g. "pad" for (pad ...).
    pub fn head(&self) -> &str {
        match self.items.first() {
            Some(Sx::Atom(s)) => s,
            _ => "",
        }
    }
    /// Child lists with the given head.
    pub fn all<'a, 'h>(&'a self, head: &'h str) -> impl Iterator<Item = &'a SList> + use<'a, 'h> {
        self.items.iter().filter_map(move |x| match x {
            Sx::List(l) if l.head() == head => Some(l),
            _ => None,
        })
    }
    pub fn get(&self, head: &str) -> Option<&SList> {
        self.all(head).next()
    }
    pub fn lists(&self) -> impl Iterator<Item = &SList> {
        self.items.iter().filter_map(|x| x.as_list())
    }
    /// i-th argument (after the head) as text.
    pub fn arg(&self, i: usize) -> Option<&str> {
        self.items.get(i + 1).and_then(|x| x.text())
    }
    pub fn argf(&self, i: usize) -> Option<f64> {
        self.items.get(i + 1).and_then(|x| x.num())
    }
    /// Value of a (key value) child, e.g. (width 0.25) -> 0.25
    pub fn valf(&self, key: &str) -> Option<f64> {
        self.get(key).and_then(|l| l.argf(0))
    }
    pub fn vals(&self, key: &str) -> Option<&str> {
        self.get(key).and_then(|l| l.arg(0))
    }
    /// Does the list contain a bare atom (flag) such as `locked`?
    pub fn has_atom(&self, a: &str) -> bool {
        self.items.iter().skip(1).any(|x| matches!(x, Sx::Atom(s) if s == a))
    }
    /// (key yes) style booleans, also accepts a bare flag atom.
    pub fn flag(&self, key: &str) -> bool {
        if self.has_atom(key) {
            return true;
        }
        matches!(self.vals(key), Some("yes") | Some("true"))
    }
    /// All string arguments (atoms/strings) after head.
    pub fn texts(&self) -> Vec<&str> {
        self.items.iter().skip(1).filter_map(|x| x.text()).collect()
    }
}

pub fn parse(src: &str) -> Result<SList> {
    let b = src.as_bytes();
    let mut i = 0usize;
    let mut stack: Vec<(Vec<Sx>, usize)> = Vec::new();
    let mut root: Option<SList> = None;
    while i < b.len() {
        let c = b[i];
        match c {
            b'(' => {
                stack.push((Vec::new(), i));
                i += 1;
            }
            b')' => {
                let (items, start) = match stack.pop() {
                    Some(x) => x,
                    None => bail!("unbalanced ')' at byte {i}"),
                };
                let l = SList { items, span: (start, i + 1) };
                i += 1;
                match stack.last_mut() {
                    Some((parent, _)) => parent.push(Sx::List(l)),
                    None => {
                        root = Some(l);
                        break;
                    }
                }
            }
            b'"' => {
                i += 1;
                let mut s = Vec::new();
                while i < b.len() && b[i] != b'"' {
                    if b[i] == b'\\' && i + 1 < b.len() {
                        i += 1;
                        let e = b[i];
                        s.push(match e {
                            b'n' => b'\n',
                            b't' => b'\t',
                            b'r' => b'\r',
                            other => other,
                        });
                    } else {
                        s.push(b[i]);
                    }
                    i += 1;
                }
                i += 1; // closing quote
                let st = String::from_utf8_lossy(&s).into_owned();
                match stack.last_mut() {
                    Some((parent, _)) => parent.push(Sx::Str(st)),
                    None => bail!("string outside list"),
                }
            }
            b' ' | b'\t' | b'\r' | b'\n' => i += 1,
            _ => {
                let st = i;
                while i < b.len() && !matches!(b[i], b' ' | b'\t' | b'\r' | b'\n' | b'(' | b')' | b'"') {
                    i += 1;
                }
                let a = String::from_utf8_lossy(&b[st..i]).into_owned();
                match stack.last_mut() {
                    Some((parent, _)) => parent.push(Sx::Atom(a)),
                    None => bail!("atom outside list"),
                }
            }
        }
    }
    match root {
        Some(r) => Ok(r),
        None => bail!("empty or unterminated s-expression"),
    }
}

/// Quote a string for KiCad output.
pub fn quote(s: &str) -> String {
    let mut o = String::with_capacity(s.len() + 2);
    o.push('"');
    for ch in s.chars() {
        match ch {
            '"' => o.push_str("\\\""),
            '\\' => o.push_str("\\\\"),
            '\n' => o.push_str("\\n"),
            c => o.push(c),
        }
    }
    o.push('"');
    o
}

/// Format a millimetre value the way KiCad does (up to 6 decimals, trimmed).
pub fn fmt_mm(v: f64) -> String {
    let mut s = format!("{:.6}", v);
    if s.contains('.') {
        while s.ends_with('0') {
            s.pop();
        }
        if s.ends_with('.') {
            s.pop();
        }
    }
    if s == "-0" {
        s = "0".into();
    }
    s
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn basic() {
        let s = r#"(kicad_pcb (version 1) (net 1 "A \"x\"") (segment (start 1 2) (end 3 4)))"#;
        let r = parse(s).unwrap();
        assert_eq!(r.head(), "kicad_pcb");
        let n = r.get("net").unwrap();
        assert_eq!(n.arg(1), Some("A \"x\""));
        let seg = r.get("segment").unwrap();
        assert_eq!(&s[seg.span.0..seg.span.1], "(segment (start 1 2) (end 3 4))");
        assert_eq!(fmt_mm(1.250000), "1.25");
    }
}
