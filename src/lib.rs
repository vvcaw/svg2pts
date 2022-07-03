use kurbo::common::solve_quadratic; // usvg already uses kurbo
use lyon_geom::cubic_bezier::CubicBezierSegment;
use lyon_geom::euclid::Vector2D;
use std::fs::File;
use std::io;
use std::io::prelude::*;
use usvg::prelude::*;
use usvg::{NodeKind, Options, PathSegment, TransformedPath, Tree};

mod errors {
    use error_chain::error_chain;
    error_chain! {}
}
use errors::*;
type Pt = Vector2D<f64, lyon_geom::euclid::UnknownUnit>;

struct PathWriter {
    start: Pt,
    current: Pt,
    last: Pt,
    accuracy: f64,
    target_dist: f64,
    pts: Vec<Pt>,
    height: f64,
}

impl PathWriter {
    fn new(target_dist: f64, accuracy: f64, height: f64) -> PathWriter {
        PathWriter {
            target_dist,
            start: Pt::default(),
            current: Pt::default(),
            last: Pt::default(),
            pts: vec![],
            accuracy,
            height,
        }
    }

    fn write_pt(&mut self, pt: Pt) {
        self.pts.push(Pt::new(pt.x, self.height - pt.y));
    }

    fn move_to(&mut self, pt: Pt) {
        self.start = pt;
        self.current = pt;
        self.last = pt;
        self.write_pt(pt)
    }

    fn write_path(&mut self, path: impl Iterator<Item = PathSegment>) -> io::Result<()> {
        use PathSegment::*;

        for seg in path {
            match seg {
                MoveTo { x, y } => {
                    self.move_to((x, y).into());
                }
                LineTo { x, y } => {
                    self.line_to((x, y).into());
                }
                ClosePath => {
                    self.close_path();
                }
                CurveTo {
                    x1,
                    y1,
                    x2,
                    y2,
                    x,
                    y,
                } => {
                    let bez = CubicBezierSegment {
                        from: (self.last.x, self.last.y).into(),
                        ctrl1: (x1, y1).into(),
                        ctrl2: (x2, y2).into(),
                        to: (x, y).into(),
                    };
                    for pt in bez.flattened(self.accuracy) {
                        self.line_to(pt.to_vector());
                    }
                }
            }
        }
        Ok(())
    }
    /// Segments Line into distance lengthed segments
    fn line_to(&mut self, line_end: Pt) {
        if self.target_dist == 0.0 {
            self.last = line_end; //record last
            self.write_pt(line_end);
        }

        // Find point on line (self.last, line_end) such that is
        // target_dist away from self.current
        let w = line_end - self.current;
        let v = self.last - line_end;
        let c = w.square_length() - self.target_dist * self.target_dist;
        if c < 0.0 {
            // line_end is two close
            self.last = line_end;
        }

        let intersect = solve_quadratic(c, 2.0 * (v.dot(w)), v.square_length());

        let mut t_min = 2.0;
        for t in intersect {
            if t >= -0.000001 && t <= 1.000001 && t < t_min {
                t_min = t;
            }
        }

        if t_min <= 1.0 {
            self.current = line_end.lerp(self.last, t_min);
            self.write_pt(self.current);
        } else {
            // line_end is two close; shouldn't happen since we
            // checked above we have extended bounds on t
            // to account for numerical imprecision
            self.last = line_end;
        }

        self.last = line_end; //record last

        let line_dist = (self.current - line_end).length();
        if line_dist < self.target_dist {
            return;
        }

        let td = self.target_dist / line_dist;

        let line_start = self.current;
        for i in 1.. {
            let t = (i as f64) * td;
            if t >= 1.0 {
                break;
            }
            self.current = line_start.lerp(line_end, t);
            self.write_pt(self.current);
        }
    }

    fn close_path(&mut self) {
        self.line_to(self.start)
    }
}

fn path_distance(acc: f64, paths: impl Iterator<Item = PathSegment>) -> f64 {
    use PathSegment::*;
    let mut last = (0.0, 0.0);
    let mut start = (0.0, 0.0);
    let mut dist = 0.0;
    for seg in paths {
        match seg {
            MoveTo { x, y } => {
                last = (x, y);
                start = last;
            }
            LineTo { x, y } => {
                dist += (Pt::new(x, y) - Pt::from(last)).length();
                last = (x, y);
            }
            ClosePath => {
                dist += (Pt::from(start) - Pt::from(last)).length();
            }
            CurveTo {
                x1,
                y1,
                x2,
                y2,
                x,
                y,
            } => {
                let bez = CubicBezierSegment {
                    from: last.into(),
                    ctrl1: (x1, y1).into(),
                    ctrl2: (x2, y2).into(),
                    to: (x, y).into(),
                };
                dist += bez.approximate_length(acc);
                last = (x, y);
            }
        }
    }
    dist
}

use std::rc::Rc;
use usvg::PathData;
use usvg::Transform;

fn extract_paths(svg: &Tree) -> Vec<(Rc<PathData>, Transform)> {
    let mut paths = Vec::default();
    for node in svg.root().descendants() {
        if let NodeKind::Path(ref path) = *node.borrow() {
            if path.fill.is_some() || path.stroke.is_some() {
                paths.push((path.data.clone(), node.transform()));
            }
        }
    }
    paths
}

pub fn get_path_from_file(
    filename: &str,
    point_number: u64,
    point_distance: f64,
) -> Vec<(f64, f64)> {
    let mut svg_buf = Vec::default();

    File::open(filename)
        .chain_err(|| "Failed to open input")
        .unwrap()
        .read_to_end(&mut svg_buf)
        .chain_err(|| "Failed to read input")
        .unwrap();

    let tree = Tree::from_data(&svg_buf, &Options::default())
        .chain_err(|| "unable to parse svg")
        .unwrap();

    let paths = extract_paths(&tree);

    let height = tree.svg_node().view_box.rect.height();

    let distance = if point_number > 0 {
        let path_distance: f64 = paths
            .iter()
            .map(|(path, transform)| path_distance(0.05, TransformedPath::new(path, *transform)))
            .sum();
        path_distance / (point_number as f64)
    } else {
        point_distance
    };

    let accuracy = if distance == 0.0 {
        0.05
    } else {
        distance / 25.0
    };

    let mut writer = PathWriter::new(distance, accuracy, height);

    for (path, transform) in &paths {
        writer
            .write_path(TransformedPath::new(path, *transform))
            .chain_err(|| "failed writing points").unwrap();
    }

    writer.pts.iter().map(|pt| (pt.x, pt.y)).collect()
}
