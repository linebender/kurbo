This directory includes the LaTeX source and MathML output of math displayed in the kurbo inline docs.

To build:

 1. Find the snipped that you want to copy
 2. Go to https://temml.org/ and set output to `MathML`
 3. Copy and paste your snippet into the input
 4. Copy the output into a file in this folder
 5. Do `#[doc = include_str!("relative/path/to/the/mathml")]` in kurbo source to inline the math.

 It's a manual process at the moment, but given that the math isn't going to be updated often it works for me.