/*
Copyright 2017 Takashi Ogura

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/
use k;
use std::io;
use urdf_rs;

#[derive(Debug)]
pub enum CollisionPart {
    Start,
    End,
}

#[derive(Debug, Fail)]
/// Error for `gear`
pub enum Error {
    #[fail(display = "{:?}", error)]
    Other { error: String },
    #[fail(display = "Collision error: {:?} is colliding", part)]
    Collision { part: CollisionPart },
    #[fail(display = "IO error {:?}", error)]
    Io { error: io::Error },
    #[fail(display = "URDF error: {:?}", error)]
    Urdf { error: urdf_rs::UrdfError },
    #[fail(display = "IK error: {:?}", error)]
    Ik { error: k::IKError },
    #[fail(display = "Joint error: {:?}", error)]
    Joint { error: k::JointError },
}

/// Result for `gear`
pub type Result<T> = ::std::result::Result<T, Error>;

impl From<io::Error> for Error {
    fn from(error: io::Error) -> Error {
        Error::Io { error }
    }
}

impl<'a> From<&'a str> for Error {
    fn from(err: &'a str) -> Error {
        Error::Other {
            error: err.to_owned(),
        }
    }
}
impl From<String> for Error {
    fn from(error: String) -> Error {
        Error::Other { error }
    }
}

impl From<urdf_rs::UrdfError> for Error {
    fn from(error: urdf_rs::UrdfError) -> Error {
        Error::Urdf { error }
    }
}

impl From<k::IKError> for Error {
    fn from(error: k::IKError) -> Error {
        Error::Ik { error }
    }
}

impl From<k::JointError> for Error {
    fn from(error: k::JointError) -> Error {
        Error::Joint { error }
    }
}
