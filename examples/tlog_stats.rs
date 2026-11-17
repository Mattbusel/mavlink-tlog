//! Prints a summary of a .tlog: senders, loss, message rates, damage.
//!
//! ```text
//! cargo run --release --example tlog_stats -- flight.tlog
//! ```

use mavlink_tlog::{message_info, LinkStats, TlogReader};

fn main() -> std::io::Result<()> {
    let path = std::env::args()
        .nth(1)
        .expect("usage: tlog_stats <file.tlog>");
    let mut log = TlogReader::new(std::io::BufReader::new(std::fs::File::open(&path)?));
    let mut stats = LinkStats::default();
    while let Some(rec) = log.next_record()? {
        stats.observe(rec.timestamp_us, &rec.frame);
    }
    let read = log.stats();
    println!(
        "{path}: {} frames, {} bytes skipped, {} bad checksums",
        read.records, read.skipped_bytes, read.bad_crc
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
        let mut by: Vec<_> = s.by_msg_id.iter().collect();
        by.sort_by(|a, b| b.1.cmp(a.1));
        for (id, n) in by.into_iter().take(8) {
            let name = message_info(*id).map_or("?", |i| i.name);
            let rate = s
                .rate_hz(*id)
                .map_or(String::from("-"), |r| format!("{r:.1} Hz"));
            println!("  {name:<24} {n:>8}  {rate}");
        }
    }
    Ok(())
}
