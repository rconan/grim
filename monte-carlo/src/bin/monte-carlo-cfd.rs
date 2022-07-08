use serde::Deserialize;
use std::{
    collections::HashMap,
    io::{self, Write},
    path::Path,
    process::Command,
};

#[derive(Deserialize, Debug)]
#[allow(dead_code)]
pub struct MonteCarlo {
    index: usize,
    #[serde(rename = "elevation[deg]")]
    elevation: f64,
    #[serde(rename = "azimuth[deg]")]
    azimuth: f64,
    #[serde(rename = "wind[m/s]")]
    wind_velocity: f64,
    #[serde(rename = "r0[m]")]
    r0: f64,
    #[serde(rename = "CFD case")]
    cfd_case: String,
    #[serde(rename = "FEM elevation")]
    fem_elevation: i32,
}

fn main() -> anyhow::Result<()> {
    let data_repo = Path::new("/fsx/grim");
    let mut rdr = csv::Reader::from_path(data_repo.join("monte-carlo.csv"))?;
    let mut cfd_cases = vec![];
    for result in rdr.deserialize() {
        let monte: MonteCarlo = result?;
        cfd_cases.push(monte.cfd_case);
    }

    cfd_cases.sort();
    println!("CFD cases: {}", cfd_cases.len());
    cfd_cases.dedup();
    println!("Unique CFD cases: {}", cfd_cases.len());

    let cfd_path = Path::new("/fsx/CASES");
    let cfd_cases: Vec<_> = cfd_cases
        .into_iter()
        .filter(|case| !cfd_path.join(case).join("optvol").is_dir())
        .collect();
    println!("Unique CFD cases w/o optvol: {}", cfd_cases.len());
    println!("{:}", cfd_cases.join(" "));

    let mut map: HashMap<u32, Vec<String>> = HashMap::new();

    for case in cfd_cases {
        /*         println!("{}", case.clone());
               let job_env = format!(
                   r#"environment='[{{name=CFD_CASE,value={}}},{{name=AWS_ACCESS_KEY_ID,value=}},{{name=AWS_SECRET_ACCESS_KEY,value=}}]'"#,
                   case.clone()
               );
               println!("{job_env}");
               let output = Command::new("aws")
                   .arg("batch")
                   .arg("submit-job")
                   .arg("--job-name")
                   .arg(format!(" {}", case.clone()))
                   .arg("--job-queue")
                   .arg("CFDJobQueue")
                   .arg("--job-definition")
                   .arg("CFDJob:6")
                   .arg("--region")
                   .arg("us-west-2")
                   .arg("--array-properties")
                   .arg("size=2001")
                   .arg("--container-overrides")
                   .arg(job_env)
                   .output()?;
               println!("status: {}", output.status);
               io::stdout().write_all(&output.stdout).unwrap();
               io::stderr().write_all(&output.stderr).unwrap();
               assert!(output.status.success());
        */
        match case {
            case if case.ends_with("OS2") => {
                map.entry(2).or_insert(Vec::new()).push(case);
            }
            case if case.ends_with("OS7") => {
                map.entry(7).or_insert(Vec::new()).push(case);
            }
            case if case.ends_with("12") => {
                map.entry(12).or_insert(Vec::new()).push(case);
            }
            case if case.ends_with("17") => {
                map.entry(17).or_insert(Vec::new()).push(case);
            }
            _ => unimplemented!(),
        }
    }
    println!("{:#?}", map);
    Ok(())
}
