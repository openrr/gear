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
use std::io;
use std::fmt;
use urdf_rs;
use k;


#[derive(Debug)]
/// Error for `gear`
pub enum Error {
    Other(String),
    Collision(String),
    Io(io::Error),
    Urdf(urdf_rs::UrdfError),
    Ik(k::IKError),
    Joint(k::JointError),
}

/// Result for `gear`
pub type Result<T> = ::std::result::Result<T, Error>;

impl From<io::Error> for Error {
    fn from(err: io::Error) -> Error {
        Error::Io(err)
    }
}

impl<'a> From<&'a str> for Error {
    fn from(err: &'a str) -> Error {
        Error::Other(err.to_owned())
    }
}
impl From<String> for Error {
    fn from(err: String) -> Error {
        Error::Other(err)
    }
}

impl From<urdf_rs::UrdfError> for Error {
    fn from(err: urdf_rs::UrdfError) -> Error {
        Error::Urdf(err)
    }
}

impl From<k::IKError> for Error {
    fn from(err: k::IKError) -> Error {
        Error::Ik(err)
    }
}

impl From<k::JointError> for Error {
    fn from(err: k::JointError) -> Error {
        Error::Joint(err)
    }
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match *self {
            Error::Other(ref msg) => write!(f, "{}", &msg),
            Error::Collision(ref msg) => write!(f, "collision detected: {}", msg),
            Error::Io(ref error) => error.fmt(f),
            Error::Urdf(ref error) => error.fmt(f),
            Error::Ik(ref error) => error.fmt(f),
            Error::Joint(ref error) => error.fmt(f),
        }
    }
}

impl ::std::error::Error for Error {
    fn description(&self) -> &str {
        match *self {
            Error::Other(ref msg) => msg.as_ref(),
            Error::Collision(ref msg) => msg.as_ref(),
            Error::Io(ref error) => error.description(),
            Error::Urdf(ref error) => error.description(),
            Error::Ik(ref error) => error.description(),
            Error::Joint(ref error) => error.description(),
        }
    }
}
