use anyhow::anyhow;
use std::{fmt, iter, slice, vec};

/// A vector that has at least 1 element.
///
/// It stores the first element on the stack, and the rest on the heap.
///
/// You can create a new `OneVec` either from the first element ([`single`][OneVec::single]) or
/// from the `TryFrom<Vec<T>>` implementation.
#[derive(Debug, Clone)]
pub struct OneVec<T> {
    /// The first, required element in the `OneVec`.
    pub first: T,
    /// The second and subsequent elements in this `OneVec` (all optional).
    pub rest: Vec<T>,
}

impl<T> OneVec<T> {
    /// Create a `OneVec` from a single element.
    pub fn single(val: T) -> Self {
        Self {
            first: val,
            rest: vec![],
        }
    }

    /// Iterate over the values in this `OneVec`.
    ///
    /// The iterator is statically guaranteed to produce at least one element.
    pub fn iter(&self) -> iter::Chain<iter::Once<&T>, slice::Iter<'_, T>> {
        self.into_iter()
    }

    /// Get the element at the given index.
    ///
    /// If the index is `0`, then this is guaranteed to return `Some`.
    pub fn get(&self, idx: usize) -> Option<&T> {
        if idx == 0 {
            Some(&self.first)
        } else {
            self.rest.get(idx - 1)
        }
    }

    /// Get the element at the given index.
    ///
    /// If the index is `0`, then this is guaranteed to return `Some`.
    pub fn get_mut(&mut self, idx: usize) -> Option<&mut T> {
        if idx == 0 {
            Some(&mut self.first)
        } else {
            self.rest.get_mut(idx - 1)
        }
    }

    /// Get the first element.
    pub fn first(&self) -> &T {
        &self.first
    }

    /// Get the first element.
    pub fn first_mut(&mut self) -> &mut T {
        &mut self.first
    }

    /// Splits the `OneVec` into the first element and the rest.
    pub fn split(&self) -> (&T, &[T]) {
        (&self.first, &self.rest)
    }

    /// Write out the vector with spaces between each element
    pub(crate) fn write_spaced(
        &self,
        mut cb: impl FnMut(&T, &mut fmt::Formatter) -> fmt::Result,
        f: &mut fmt::Formatter,
    ) -> fmt::Result {
        cb(&self.first, f)?;
        for v in &self.rest {
            cb(v, f)?;
        }
        Ok(())
    }
}

impl<T> TryFrom<Vec<T>> for OneVec<T> {
    type Error = anyhow::Error;
    fn try_from(mut v: Vec<T>) -> Result<Self, Self::Error> {
        // Annoyingly the `Vec::remove` method can panic, so we have to check
        // the vec is non-empty
        if v.is_empty() {
            return Err(anyhow!("vector must not be empty"));
        }
        let first = v.remove(0);
        Ok(OneVec { first, rest: v })
    }
}

impl<'a, T> IntoIterator for &'a OneVec<T> {
    type IntoIter = iter::Chain<iter::Once<&'a T>, slice::Iter<'a, T>>;
    type Item = &'a T;

    fn into_iter(self) -> Self::IntoIter {
        iter::once(&self.first).chain(&self.rest)
    }
}

impl<T> IntoIterator for OneVec<T> {
    type IntoIter = iter::Chain<iter::Once<T>, vec::IntoIter<T>>;
    type Item = T;

    fn into_iter(self) -> Self::IntoIter {
        iter::once(self.first).chain(self.rest)
    }
}
