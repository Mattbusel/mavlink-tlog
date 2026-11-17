# Testing

```text
bash tools/fetch_logs.sh      # two public logs, pinned by commit and sha256
cargo test --release
cargo test --no-default-features --lib
cargo build --no-default-features --target thumbv7em-none-eabihf
```

Without the downloaded logs, their tests print `SKIP` and pass; the
committed rust-mavlink log is always tested.

## Reference numbers

`tests/data/*.pymavlink.json` come from `tools/pymavlink_reference.py`
(pymavlink 2.4.50, `all` dialect, `robust_parsing=True`):

```text
pip install pymavlink==2.4.50
python tools/pymavlink_reference.py tests/data/external/pymavlink-flight.tlog > tests/data/pymavlink-flight.pymavlink.json
```

`src/table.rs` (message ids, CRC_EXTRA, lengths, names) comes from
`tools/gen_table.py`. CI regenerates both and fails on any difference.
