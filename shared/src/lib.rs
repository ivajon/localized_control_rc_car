//! Defines some shared dependencies such as
//! communications protocols and the methods for parsing them.

#![cfg_attr(all(feature = "no_std", not(test)), no_std)]
#![deny(clippy::all)]
#![deny(rustdoc::all)]
#![deny(warnings)]
#![feature(iter_next_chunk)]
#![feature(associated_type_defaults)]
#![feature(generic_const_exprs)]
#![allow(incomplete_features)]
#![allow(clippy::result_unit_err)]

pub mod controller;
pub mod gain_scheduling;
pub mod protocol;

#[derive(Clone)]
pub struct OwnedItterator<Item: Sized + Clone, const SIZE: usize> {
    buff: [Item; SIZE],
    ptr: usize,
}

impl<Item: Sized + Clone, const SIZE: usize> OwnedItterator<Item, SIZE> {
    fn new(buff: [Item; SIZE]) -> Self {
        Self { buff, ptr: 0 }
    }
}
impl<Item: Sized + Clone, const SIZE: usize> Iterator for OwnedItterator<Item, SIZE> {
    type Item = Item;

    fn next(&mut self) -> Option<Self::Item> {
        if self.ptr >= SIZE {
            return None;
        }
        let item = self.buff[self.ptr].clone();
        self.ptr += 1;
        Some(item)
    }
}
