use predicates::prelude::*;

#[test]
fn help_works() {
    let mut cmd = assert_cmd::cargo::cargo_bin_cmd!("termviz");
    cmd.arg("--help")
        .assert()
        .success()
        .stdout(predicate::str::contains("ROS visualization on the terminal"));
}
