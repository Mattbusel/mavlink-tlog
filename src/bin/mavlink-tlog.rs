//! `mavlink-tlog`: inspect, export, cut and check MAVLink .tlog files.
//!
//! ```text
//! mavlink-tlog stats   FILE [filters]
//! mavlink-tlog csv     FILE [filters] [--follow]
//! mavlink-tlog json    FILE [filters] [--follow]
//! mavlink-tlog slice   FILE OUT [filters]
//! mavlink-tlog signing FILE --key HEX [filters]
//! ```

use std::collections::BTreeMap;
use std::fs::File;
use std::io::{self, BufReader, BufWriter, Write};
use std::process::ExitCode;
use std::time::Duration;

use mavlink_tlog::{
    message_info, FrameRef, LinkStats, SignatureStatus, SigningVerifier, TlogReader, TlogWriter,
    Version,
};

const USAGE: &str = "\
mavlink-tlog: inspect, export, cut and check MAVLink .tlog files

USAGE:
  mavlink-tlog stats   FILE [filters]           senders, loss, rates, longest dropout, damage
  mavlink-tlog csv     FILE [filters] [--follow] one CSV row per frame on stdout
  mavlink-tlog json    FILE [filters] [--follow] one JSON object per frame per line on stdout
  mavlink-tlog slice   FILE OUT [filters]       write the matching records to a new .tlog
  mavlink-tlog signing FILE --key HEX [filters] check MAVLink 2 signatures with a 32 byte key

FILTERS:
  --from SECONDS    skip records earlier than this many seconds after the first record
  --to SECONDS      skip records later than this many seconds after the first record
  --from-us US      skip records with a timestamp (Unix microseconds) before US
  --to-us US        skip records with a timestamp (Unix microseconds) after US
  --sys ID          only this system id (repeatable)
  --comp ID         only this component id (repeatable)
  --msg NAME|ID     only this message (repeatable), for example --msg HEARTBEAT --msg 33

--follow keeps reading a log that is still being written (stop with Ctrl-C).
";

#[derive(Default)]
struct Filter {
    from_s: Option<f64>,
    to_s: Option<f64>,
    from_us: Option<u64>,
    to_us: Option<u64>,
    sys: Vec<u8>,
    comp: Vec<u8>,
    msg: Vec<u32>,
    first_us: Option<u64>,
}

impl Filter {
    /// `Some(seconds since the first record)` when the record passes.
    fn pass(&mut self, ts: u64, f: &FrameRef<'_>) -> Option<f64> {
        let first = *self.first_us.get_or_insert(ts);
        let rel = (ts as f64 - first as f64) / 1e6;
        let ok = self.from_s.map_or(true, |v| rel >= v)
            && self.to_s.map_or(true, |v| rel <= v)
            && self.from_us.map_or(true, |v| ts >= v)
            && self.to_us.map_or(true, |v| ts <= v)
            && (self.sys.is_empty() || self.sys.contains(&f.sys_id()))
            && (self.comp.is_empty() || self.comp.contains(&f.comp_id()))
            && (self.msg.is_empty() || self.msg.contains(&f.msg_id()));
        ok.then_some(rel)
    }
}

struct Args {
    cmd: String,
    files: Vec<String>,
    filter: Filter,
    follow: bool,
    key: Option<[u8; 32]>,
}

fn parse_msg(v: &str) -> Result<u32, String> {
    if let Ok(id) = v.parse::<u32>() {
        return Ok(id);
    }
    let want = v.to_ascii_uppercase();
    (0..=0xFF_FFFF)
        .find(|&id| message_info(id).is_some_and(|i| i.name == want))
        .ok_or_else(|| format!("unknown message {v}"))
}

fn parse_key(v: &str) -> Result<[u8; 32], String> {
    let v = v.trim();
    if v.len() != 64 || !v.is_ascii() {
        return Err("--key needs 64 hex digits (32 bytes)".into());
    }
    let mut key = [0u8; 32];
    for (i, k) in key.iter_mut().enumerate() {
        *k = u8::from_str_radix(&v[2 * i..2 * i + 2], 16)
            .map_err(|_| "--key needs 64 hex digits (32 bytes)".to_string())?;
    }
    Ok(key)
}

fn parse_args() -> Result<Args, String> {
    let mut it = std::env::args().skip(1);
    let cmd = it.next().ok_or("")?;
    let mut a = Args {
        cmd,
        files: Vec::new(),
        filter: Filter::default(),
        follow: false,
        key: None,
    };
    while let Some(arg) = it.next() {
        let mut val = |name: &str| it.next().ok_or(format!("{name} needs a value"));
        let num = |v: String| v.parse::<f64>().map_err(|_| format!("not a number: {v}"));
        let int = |v: String| v.parse::<u64>().map_err(|_| format!("not an integer: {v}"));
        let id = |v: String| v.parse::<u8>().map_err(|_| format!("not an id 0-255: {v}"));
        match arg.as_str() {
            "--from" => a.filter.from_s = Some(num(val("--from")?)?),
            "--to" => a.filter.to_s = Some(num(val("--to")?)?),
            "--from-us" => a.filter.from_us = Some(int(val("--from-us")?)?),
            "--to-us" => a.filter.to_us = Some(int(val("--to-us")?)?),
            "--sys" => a.filter.sys.push(id(val("--sys")?)?),
            "--comp" => a.filter.comp.push(id(val("--comp")?)?),
            "--msg" => a.filter.msg.push(parse_msg(&val("--msg")?)?),
            "--key" => a.key = Some(parse_key(&val("--key")?)?),
            "--follow" => a.follow = true,
            "-h" | "--help" => return Err(String::new()),
            s if s.starts_with("--") => return Err(format!("unknown option {s}")),
            _ => a.files.push(arg),
        }
    }
    Ok(a)
}

fn main() -> ExitCode {
    let args = match parse_args() {
        Ok(a) => a,
        Err(e) => {
            if !e.is_empty() {
                eprintln!("error: {e}\n");
            }
            eprint!("{USAGE}");
            return ExitCode::from(2);
        }
    };
    let need = if args.cmd == "slice" { 2 } else { 1 };
    if args.files.len() != need {
        eprint!("{USAGE}");
        return ExitCode::from(2);
    }
    let result = match args.cmd.as_str() {
        "stats" => stats(args),
        "csv" => export(args, false),
        "json" => export(args, true),
        "slice" => slice(args),
        "signing" => signing(args),
        "-h" | "--help" | "help" => {
            print!("{USAGE}");
            Ok(())
        }
        other => {
            eprintln!("error: unknown command {other}\n");
            eprint!("{USAGE}");
            return ExitCode::from(2);
        }
    };
    match result {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) if e.kind() == io::ErrorKind::BrokenPipe => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("error: {e}");
            ExitCode::FAILURE
        }
    }
}

fn open(path: &str, follow: bool) -> io::Result<TlogReader<BufReader<File>>> {
    let f = BufReader::new(File::open(path)?);
    Ok(if follow {
        TlogReader::follow(f)
    } else {
        TlogReader::new(f)
    })
}

fn name(id: u32) -> String {
    message_info(id).map_or_else(|| format!("UNKNOWN_{id}"), |i| i.name.to_string())
}

fn hex(bytes: &[u8]) -> String {
    use std::fmt::Write as _;
    let mut s = String::with_capacity(bytes.len() * 2);
    for b in bytes {
        let _ = write!(s, "{b:02x}");
    }
    s
}

fn stats(mut a: Args) -> io::Result<()> {
    let path = &a.files[0];
    let mut log = open(path, false)?;
    let mut stats = LinkStats::default();
    let mut matched = 0u64;
    while let Some(rec) = log.next_record()? {
        if a.filter.pass(rec.timestamp_us, &rec.frame).is_some() {
            stats.observe(rec.timestamp_us, &rec.frame);
            matched += 1;
        }
    }
    let read = log.stats();
    println!(
        "{path}: {} frames ({matched} matched), {} bytes skipped, {} bad checksums, {} bytes truncated at the end",
        read.records, read.skipped_bytes, read.bad_crc, read.truncated_tail
    );
    for ((sys, comp), s) in stats.sources() {
        let secs = (s.last_us - s.first_us) as f64 / 1e6;
        println!(
            "\nsender {sys}/{comp}: {} frames over {secs:.1} s, {} lost ({:.2}%), {} duplicates, {} out of order",
            s.received,
            s.lost,
            s.loss_ratio() * 100.0,
            s.duplicates,
            s.out_of_order
        );
        if s.longest_gap_us > 0 {
            let first = a.filter.first_us.unwrap_or(s.first_us);
            println!(
                "  longest silence {:.3} s, ending at +{:.3} s",
                s.longest_gap_us as f64 / 1e6,
                (s.longest_gap_end_us - first) as f64 / 1e6
            );
        }
        let mut by: Vec<_> = s.by_msg_id.iter().collect();
        by.sort_by(|a, b| b.1.cmp(a.1).then(a.0.cmp(b.0)));
        for (id, n) in by {
            let rate = s
                .rate_hz(*id)
                .map_or(String::from("-"), |r| format!("{r:.2} Hz"));
            println!("  {:<32} {n:>9}  {rate}", name(*id));
        }
    }
    Ok(())
}

fn export(mut a: Args, json: bool) -> io::Result<()> {
    let mut log = open(&a.files[0], a.follow)?;
    let out = io::stdout();
    let mut out = BufWriter::new(out.lock());
    if !json {
        writeln!(
            out,
            "timestamp_us,time_s,sys_id,comp_id,seq,msg_id,msg_name,version,payload_len,signed,payload_hex"
        )?;
    }
    loop {
        while let Some(rec) = log.next_record()? {
            let f = rec.frame;
            let Some(rel) = a.filter.pass(rec.timestamp_us, &f) else {
                continue;
            };
            let v = match f.version() {
                Version::V1 => 1,
                Version::V2 => 2,
            };
            if json {
                writeln!(
                    out,
                    "{{\"timestamp_us\":{},\"time_s\":{rel:.6},\"sys_id\":{},\"comp_id\":{},\"seq\":{},\"msg_id\":{},\"msg_name\":\"{}\",\"version\":{v},\"payload_len\":{},\"signed\":{},\"payload_hex\":\"{}\"}}",
                    rec.timestamp_us,
                    f.sys_id(),
                    f.comp_id(),
                    f.seq(),
                    f.msg_id(),
                    name(f.msg_id()),
                    f.payload_len(),
                    f.signature().is_some(),
                    hex(f.payload())
                )?;
            } else {
                writeln!(
                    out,
                    "{},{rel:.6},{},{},{},{},{},{v},{},{},{}",
                    rec.timestamp_us,
                    f.sys_id(),
                    f.comp_id(),
                    f.seq(),
                    f.msg_id(),
                    name(f.msg_id()),
                    f.payload_len(),
                    f.signature().is_some(),
                    hex(f.payload())
                )?;
            }
        }
        if !a.follow {
            break;
        }
        out.flush()?;
        std::thread::sleep(Duration::from_millis(200));
    }
    out.flush()?;
    report_damage(&log);
    Ok(())
}

fn report_damage<R>(log: &TlogReader<R>)
where
    R: io::Read,
{
    let s = log.stats();
    if s.skipped_bytes > 0 || s.truncated_tail > 0 {
        eprintln!(
            "note: {} bytes skipped ({} bad checksums), {} bytes truncated at the end",
            s.skipped_bytes, s.bad_crc, s.truncated_tail
        );
    }
}

fn slice(mut a: Args) -> io::Result<()> {
    let mut log = open(&a.files[0], false)?;
    let mut w = TlogWriter::new(BufWriter::new(File::create(&a.files[1])?));
    while let Some(rec) = log.next_record()? {
        if a.filter.pass(rec.timestamp_us, &rec.frame).is_some() {
            w.write_frame(rec.timestamp_us, rec.frame.as_bytes())?;
        }
    }
    w.flush()?;
    eprintln!(
        "{}: wrote {} of {} records",
        a.files[1],
        w.records(),
        log.stats().records
    );
    report_damage(&log);
    Ok(())
}

#[derive(Default)]
struct SignCounts {
    valid: u64,
    bad: u64,
    replay: u64,
    unsigned: BTreeMap<u32, u64>,
}

fn signing(mut a: Args) -> io::Result<()> {
    let key = a.key.ok_or_else(|| {
        io::Error::new(
            io::ErrorKind::InvalidInput,
            "signing needs --key with 64 hex digits",
        )
    })?;
    let mut v = SigningVerifier::new(key);
    let mut log = open(&a.files[0], false)?;
    let mut by: BTreeMap<(u8, u8), SignCounts> = BTreeMap::new();
    while let Some(rec) = log.next_record()? {
        if a.filter.pass(rec.timestamp_us, &rec.frame).is_none() {
            continue;
        }
        let c = by
            .entry((rec.frame.sys_id(), rec.frame.comp_id()))
            .or_default();
        match v.check(&rec.frame) {
            SignatureStatus::Valid => c.valid += 1,
            SignatureStatus::BadSignature => c.bad += 1,
            SignatureStatus::Replay => c.replay += 1,
            SignatureStatus::Unsigned => *c.unsigned.entry(rec.frame.msg_id()).or_insert(0) += 1,
        }
    }
    let mut suspicious = false;
    for ((sys, comp), c) in &by {
        let unsigned: u64 = c.unsigned.values().sum();
        println!(
            "sender {sys}/{comp}: {} valid, {} bad signature, {} replayed, {unsigned} unsigned",
            c.valid, c.bad, c.replay
        );
        let signs = c.valid + c.bad + c.replay > 0;
        if signs && unsigned > 0 {
            suspicious = true;
            println!("  unsigned frames from a sender that signs:");
            for (id, n) in &c.unsigned {
                println!("    {:<32} {n:>9}", name(*id));
            }
        }
        if c.bad + c.replay > 0 {
            suspicious = true;
        }
    }
    report_damage(&log);
    if suspicious {
        println!("\nFOUND: forged, replayed or unsigned frames from signing senders");
    }
    Ok(())
}
