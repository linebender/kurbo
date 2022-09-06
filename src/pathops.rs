/// Boolean operations on BezPaths
/// 
/// The methodology approximately follows that which is described here:
/// https://losingfight.com/blog/2011/07/07/how-to-implement-boolean-operations-on-bezier-paths-part-1/
/// https://losingfight.com/blog/2011/07/08/how-to-implement-boolean-operations-on-bezier-paths-part-2/
/// https://losingfight.com/blog/2011/07/09/how-to-implement-boolean-operations-on-bezier-paths-part-3/

use crate::{Point, Rect, BezPath, PathEl, Shape};
use crate::{Image, Size, Line}; // DO NOT COMMIT
use std::sync::atomic::{AtomicUsize, Ordering}; // DO NOT COMMIT

/// An operation that can be performed on a path
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum PathOperation<'a> {
    /// Subtracts the other path from the current path
    Subtract(&'a BezPath),
    /// Like subtract but doesn't close current path along second path
    Outside(&'a BezPath),
    /// Gets the inersection of this path with another
    Intersect(&'a BezPath),
    /// Like intersection but doesn't close current path along second path
    Inside(&'a BezPath),
    /// Unites this path with another
    UniteWith(&'a BezPath),
    /// Gets the exclusive-or operation with another path
    Xor(&'a BezPath),
}

bitflags::bitflags! {
    // private struct, used in do_path_op
    struct ClosedPathOperationsFlags: u32 {
       const DISCARD_SEGS       = 0b00000000;
       const KEEP_A_OUTSIDE_B   = 0b00000001;
       const KEEP_A_INSIDE_B    = 0b00000010;
       const KEEP_B_INSIDE_A    = 0b00000100;
       const KEEP_B_OUTSIDE_A   = 0b00001000;
       const KEEP_ALL_SEGS      = Self::KEEP_A_OUTSIDE_B.bits
                                | Self::KEEP_A_INSIDE_B.bits
                                | Self::KEEP_B_INSIDE_A.bits
                                | Self::KEEP_B_OUTSIDE_A.bits;
    }
}

/// Compute an operation between two paths
pub fn path_path_operation(
    path_a: &BezPath,
    operation: PathOperation,
) -> BezPath {
    match operation {
        PathOperation::Subtract(path_b) => {
            // Subtract b from a
            do_path_op(
                path_a,
                path_b,
                ClosedPathOperationsFlags::KEEP_A_OUTSIDE_B
                | ClosedPathOperationsFlags::KEEP_B_INSIDE_A,
                true,
                0.,
            )
        }
        PathOperation::Outside(path_b) => {
            // Get a outside of b
            do_path_op(
                path_a,
                path_b,
                ClosedPathOperationsFlags::KEEP_A_OUTSIDE_B,
                false,
                0.,
            )
        }
        PathOperation::Intersect(path_b) => {
            // Get the intersections of a and b
            do_path_op(
                path_a,
                path_b,
                ClosedPathOperationsFlags::KEEP_A_INSIDE_B
                | ClosedPathOperationsFlags::KEEP_B_INSIDE_A,
                true,
                0.,
            )
        }
        PathOperation::Inside(path_b) => {
            // Get a inside of b
            do_path_op(
                path_a,
                path_b,
                ClosedPathOperationsFlags::KEEP_A_INSIDE_B,
                false,
                0.,
            )
        }
        PathOperation::UniteWith(path_b) => {
            // Units a and b
            do_path_op(
                path_a,
                path_b,
                ClosedPathOperationsFlags::KEEP_A_OUTSIDE_B
                | ClosedPathOperationsFlags::KEEP_B_OUTSIDE_A,
                true,
                0.,
            )
        }
        PathOperation::Xor(path_b) => {
            // Get a XOR b
            // We can actually do a simple shortcut with this one. All we have
            // to do is append path_b to path_a. Since they are both even-odd
            // fill, everything is taken care of just by the logic of the fill.
            let mut path_final = path_a.clone();
            path_final.extend(path_b.iter());
            path_final
        }
    }
}

#[derive(Clone, Debug)]
struct PathSectionInfo<'a> {
    /// The index in the array of elements so that we can quickly and easily
    /// modify the array
    pub index: usize,
    /// Path index, ex: 0 for path1, and 1 for path2
    pub path_index: usize,
    /// The path for the section
    pub section_path: &'a BezPath,
    /// The bounding rect of the path section
    pub section_bounds: Rect,
    /// A flag representing whether we have processed this section
    pub processed: bool,
    /// A flag determining whether we will keep this section in the final
    /// result, only valid if processed == true
    pub keep: bool,
}

impl PathSectionInfo<'_> {
    pub fn new(
        index: usize,
        path_index: usize,
        section_path: &BezPath,
    ) -> PathSectionInfo {
        PathSectionInfo {
            index: index,
            path_index: path_index,
            section_path: section_path,
            section_bounds: section_path.bounding_box(),
            processed: false,
            keep: false
        }
    }
}

/// Populates 
fn populate_elements_list<'a>(
    elements: &'a Vec<PathSectionInfo<'a>>,
    path_index: usize,
    sections: &'a Vec<BezPath>,
) -> Vec<PathSectionInfo<'a>> {
    let mut new_elements = elements.clone();
    let mut test = sections.iter()
        .enumerate()
        .map(|(index, path)| {
            // Create an info struct
            PathSectionInfo::new(
                elements.len() + index,
                path_index,
                &path,
            )
        })
        .collect::<Vec<_>>();
    new_elements.append(&mut test);
    new_elements
}

/// Determines whihc
fn flag_path_sections_to_keep(
    elements: &mut Vec<PathSectionInfo>,
    process_winding_results: &dyn Fn(Vec<(f64, i32, usize, usize, bool)>, &mut Vec<PathSectionInfo>),
    accuracy: f64,
) -> Vec<PathEl> {
    while let Some(info1) = elements.iter().find(|info| !info.processed) {
        // We've picked the first unprocessed element and now we need to draw a
        // ray through it to test if we should keep it

        // Create a ray that travels through our path
        let ray_is_horizontal = info1.section_bounds.width().abs() <= info1.section_bounds.height().abs();
        let ray_position = if ray_is_horizontal { (info1.section_bounds.min_y() + info1.section_bounds.max_y()) / 2. } else { (info1.section_bounds.min_x() + info1.section_bounds.max_x()) / 2. };
        let ray_point = if ray_is_horizontal { Point::new(info1.section_bounds.max_x() + 1., ray_position) } else { Point::new(ray_position, info1.section_bounds.max_y() + 1.) };
        let ray_fn = if ray_is_horizontal { BezPath::winding_x_results } else { BezPath::winding_y_results };

        // Loop through all paths and determine which ones intersect our ray
        let mut winding_results = elements.iter().by_ref()
            .map(|info2| {
                let winding_array = ray_fn(&*info2.section_path, ray_point).into_iter()
                    .map(|(pt, winding)| (pt, winding, info2.index.clone(), info2.path_index.clone(), info2.processed.clone()))
                    .collect::<Vec<_>>();
                winding_array
            })
            .flatten()
            .collect::<Vec<_>>();
        
        // Sort by the position of the intersection with the ray
        winding_results.sort_by(|(a, _, _, _, _), (b, _, _, _, _)| (a).partial_cmp(b).unwrap());

        process_winding_results(
            winding_results,
            elements
        );
    }

    // Remove flagged sections of the path, add a bool representing whether or
    // not the path section should be connected to the previous one in the
    // final result.
    let mut elements = elements.into_iter()
        .filter_map(|info| if info.keep { Some((info.section_path, false, false)) } else { None })
        .filter(|(path, _, _)| !path.elements().is_empty())
        .collect::<Vec<_>>();

    // Re-close paths
    for index1 in 0..elements.len() {
        if let Some(path_end1) = elements[index1].0.get_seg_end(elements[index1].0.elements().len() - 1) {
            let accuracy2 = accuracy * accuracy;
            let mut candidates = elements.iter()
                .enumerate()
                .filter(|(index2, _)| *index2 > index1)
                .map(|(index2, (path2, _, _))| (index2, path2.get_seg_end(0).unwrap(), path2.get_seg_end(path2.elements().len() - 1).unwrap()))
                .filter(|&(_, start, end)| (start - path_end1).hypot2() <= accuracy2 || (end - path_end1).hypot2() <= accuracy2)
                .collect::<Vec<_>>();
            candidates.sort_by(|&(_, start1, end1), &(_, start2, end2)| {
                let d1 = (start1 - path_end1).hypot2().min((end1 - path_end1).hypot2());
                let d2 = (start2 - path_end1).hypot2().min((end2 - path_end1).hypot2());
                d1.partial_cmp(&d2).unwrap()
            });
            
            if let Some(&(connected_index, start, end)) = candidates.get(0) {
                // Set a flag so that we connect to path1 later by removing path2's move-to
                elements[connected_index].1 = true;

                // Determine if we need to reverse the segment
                elements[connected_index].2 = (start - path_end1).hypot2() > (end - path_end1).hypot2();

                // Move index to be next in line
                elements.swap(index1 + 1, connected_index);
            }
        }
    };
    
    if !elements.is_empty() {
        assert!(!elements[0].1); // Fist element can't conenct to the previous one
    }

    elements.into_iter()
        .map(|(path, connect_to_previous, reverse)| {
            let p_rev = if reverse { path.reverse() } else { BezPath::new() };
            let p_correct = if reverse { &p_rev } else { path };
            BezPath::from_vec(p_correct.elements()[if connect_to_previous { 1 } else { 0 }..].to_vec())
        })
        .flatten()
        .collect::<Vec<_>>()
}

/// This function converts a winding-fill path into an even-odd filled path
/// Note: Since there is no winding fill property of path, the path is assumed
/// to have a winding fill when this function is called
/// Note: Open paths are closed
pub fn convert_path_to_even_odd(
    path: &BezPath,
    accuracy: f64,
) -> BezPath {
    let bounds = path.bounding_box();
    let accuracy = Point::epsilon(Point::new(bounds.max_x(), bounds.max_y()), accuracy);

    // Make a copy of the path
    let mut path = path.clone();

    // Close all subpaths because this path is treated as being closed anyways
    path.close_subpaths();

    // Break apart at self intersections
    BezPath::break_at_self_intersections(&mut path, accuracy);
    let paths = path.split_at_moves();

    // Convert the list of elements into a vector of tuples to make tracking elements easier
    let elements = Vec::<PathSectionInfo>::new();
    let elements = populate_elements_list(&elements, 0, &paths);
    let mut elements = elements; // Make mutable

    fn process_winding_results(
        winding_results: Vec<(f64, i32, usize, usize, bool)>, 
        elements: &mut Vec<PathSectionInfo>) {
            winding_results
            .into_iter()
            .scan(0, |winding_total, (_, w, index, _, processed2)| {
                let winding_before = *winding_total;
                *winding_total = *winding_total + w;
                Some((winding_before, *winding_total, index, processed2))
            })
            .for_each(|(w_before, w_current, index, _)| {
                let ref mut info = elements[index];
                
                // Discard if crossing the path didn't change whether or not we're inside the original path
                info.keep = (w_before != 0) != (w_current != 0);

                // Mark as processed
                info.processed = true;
            });
    }

    let elements = flag_path_sections_to_keep(
        &mut elements,
        &process_winding_results,
        accuracy
    );    
    BezPath::from_vec(elements)
}

/// Performs an operation on the current path
/// Note: Paths are assumed to be even-odd filled
fn do_path_op(
    path_a: &BezPath,
    path_b: &BezPath,
    closed_flags: ClosedPathOperationsFlags,
    connect_nearby_paths: bool,
    accuracy: f64,
) -> BezPath {
    let bounds = path_a.bounding_box().union(path_b.bounding_box());
    let accuracy = Point::epsilon(Point::new(bounds.max_x(), bounds.max_y()), accuracy);

    // Make copies of the paths
    let mut path_a = path_a.clone();
    let mut path_b = path_b.clone();

    if connect_nearby_paths {
        // We treat all paths as if they are closed, so we'll just go ahead and
        // close the paths
        path_a.close_subpaths();
        path_b.close_subpaths();
    }

    // Find all intersections between the two paths and break them apart at
    // those locations
    BezPath::break_at_intersections(&mut path_a, &mut path_b, accuracy);
    let paths_a = path_a.split_at_moves();
    let paths_b = path_b.split_at_moves();

    // Make a list of all elements in the paths
    let elements = Vec::<PathSectionInfo>::new();
    let elements = populate_elements_list(&elements, 0, &paths_a);
    let elements = populate_elements_list(&elements, 1, &paths_b);
    let mut elements = elements; // Make mutable

    let process_winding_results = |
        winding_results: Vec<(f64, i32, usize, usize, bool)>, 
        elements: &mut Vec<PathSectionInfo>,
        | {
            winding_results.into_iter()
                .scan((0, 0), |winding_total, (_, w, index, path_index, processed2)| {
                    if path_index == 0 { winding_total.0 = winding_total.0 + w } else { winding_total.1 = winding_total.1 + w };
                    Some((*winding_total, index, path_index, processed2))
                })
                .for_each(|(w_current, index, path_index, _)| {
                    let ref mut info = elements[index];
                    
                    // Use closed_flags to determine if we need to remove or keep the path section
                    let is_path_a = path_index == 0;
                    let inside_path_a = (w_current.0 % 2) != 0;
                    let inside_path_b = (w_current.1 % 2) != 0;
                    info.keep = (is_path_a && closed_flags.contains(ClosedPathOperationsFlags::KEEP_A_OUTSIDE_B) && !inside_path_b)
                        || (is_path_a && closed_flags.contains(ClosedPathOperationsFlags::KEEP_A_INSIDE_B) && inside_path_b)
                        || (!is_path_a && closed_flags.contains(ClosedPathOperationsFlags::KEEP_B_INSIDE_A) && inside_path_a)
                        || (!is_path_a && closed_flags.contains(ClosedPathOperationsFlags::KEEP_B_OUTSIDE_A) && !inside_path_a);


                    // Mark as processed
                    info.processed = true;
                });
    };
    
    let elements = flag_path_sections_to_keep(
        &mut elements,
        &process_winding_results,
        accuracy
    );    
    BezPath::from_vec(elements)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{Shape, Circle};
    use crate::{ParamCurve}; // DO NOT COMMIT

    #[test]
    fn test_circle_unite_circle() {
        let c1 = Circle::new((0., 0.), 10.).to_path(1e-3);
        let c2 = Circle::new((5., 0.), 10.).to_path(1e-3);
        let res = path_path_operation(&c1, PathOperation::UniteWith(&c2));

        // DO NOT COMMIT
        static COUNTER_PATHOP: AtomicUsize = AtomicUsize::new(0);
        let mut img1 = Image::from_fixed_size(Size::new(1000., 1000.), 20);
        let intersects = c1.intersections(&c2, 0.);
        for i in &intersects {
            let seg1 = c1.get_seg(i.0.0).unwrap();
            let seg2 = c2.get_seg(i.1.0).unwrap();

            img1.add_point(seg1.eval(i.0.1), 0xFF00FF, 10., true);
            img1.add_point(seg2.eval(i.1.1), 0xFF00FF, 15., false);
        }
        img1.add_path(&res, 0xFF00FF, true);
        img1.add_path(&c1, 0x800000, false);
        img1.add_path(&c2, 0x008000, false);
        img1.save(&format!("_path_op_{}.png", COUNTER_PATHOP.fetch_add(1, Ordering::Relaxed)));
        
        assert_eq!(res.elements().len(), 9);
    }

    #[test]
    fn test_convert_to_even_odd() {
        let mut path = BezPath::new();
        path.move_to((0., 0.));
        path.line_to((0., 1.));
        path.curve_to((2., 0.5), (-1., 0.5), (1., 1.));
        path.line_to((-0.5, 0.));
        path.close_path();
        
        let path2 = convert_path_to_even_odd(&path, 0.);

        assert_eq!(path2.elements().len(), 10);
    }
}
