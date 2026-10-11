fn main() {
    let args: Vec<_> = std::env::args().skip(1).collect();
    assert!(args.starts_with(&["api".into(), "--paginate".into()]));
    if std::env::var_os("PHOXAL_RELEASE_API_FAILURE").is_some() { std::process::exit(7); }
    print!("{}", std::env::var("PHOXAL_RELEASE_RECORD").unwrap_or_default());
}
