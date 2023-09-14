extern crate rayon;
extern crate walkdir;
extern crate xml;

use rayon::prelude::*;
use std::fs::File;
use std::io::BufReader;
use walkdir::WalkDir;
use xml::reader::{EventReader, XmlEvent};

fn process_single_xml(file_path: &str) -> Option<String> {
    let file = File::open(file_path).unwrap();
    let file = BufReader::new(file);
    let mut parser = EventReader::new(file);

    loop {
        let e = match parser.next() {
            Ok(event) => event,
            Err(_) => {
                println!("Failed to parse {}", file_path);
                return None;
            }
        };

        match e {
            XmlEvent::StartElement { name, .. } => {
                if name.local_name == "name" {
                    match parser.next() {
                        Ok(XmlEvent::Characters(name)) => {
                            return Some(name);
                        }
                        _ => {
                            return None;
                        }
                    }
                }
            }
            XmlEvent::EndDocument => break,
            _ => {}
        }
    }
    None
}

async fn get_package_names(src_path: &str) -> Vec<String> {
    let mut xml_files = Vec::new();
    let mut it = WalkDir::new(src_path).into_iter();

    while let Some(entry) = it.next() {
        match entry {
            Ok(e) => {
                if e.file_name().to_string_lossy() == "package.xml" {
                    xml_files.push(e.path().to_string_lossy().into_owned());
                    if let Some(..) = e.depth().checked_sub(1) {
                        it.skip_current_dir();
                    }
                }
            }
            Err(_) => {
                // Handle the error here if needed
            }
        }
    }

    xml_files
        .par_iter()
        .filter_map(|file_path| process_single_xml(file_path))
        .collect()
}
pub async fn parse_and_get_packages(src_path: &str) -> Vec<String> {
    get_package_names(src_path).await
}
