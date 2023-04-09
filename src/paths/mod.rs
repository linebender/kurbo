//! Different path representations, useful in different contexts.
//!
//! In `kurbo`, the canonical path representation is `BezPath`, which is always drawn with perfect
//! accuracy. However, sometimes it is useful to represent paths in different ways, for example
//! circular arcs are often used in architecture, and SVG defines a different path model to
//! `BezPath`, so they cannot be used interchangeably.

pub mod svg;
