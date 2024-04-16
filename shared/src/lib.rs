//! Defines some shared dependencies such as
//! communications protocols and the methods for parsing them.

#![cfg_attr(all(feature = "no_std", not(test)), no_std)]
#![deny(clippy::all)]
#![deny(rustdoc::all)]
#![deny(warnings)]

pub mod controller;
