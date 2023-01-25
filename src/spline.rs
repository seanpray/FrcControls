use std::io::Error;

pub fn generate_equation(data: String) -> Vec<f32> {
    // take in csv formatted string and parse
    let Ok(data) = Csv::from_str(&data) else {
        // if it fails don't crash just return nothing
        return vec![];
    };
    // determine rough number of constraints, we are going to recheck this after filtering out
    // potentially invalid constraints
    let constraint_size = data.constraint_size().unwrap_or(0);
    // if we have velocity at all then we know to filter out constraints that don't have velocity
    let velocity = data.columns.contains(&"velocity");
    // separate the data into the x and y components
    let (x, y) = data.split();
    let mut uv = Vec::with_capacity(constraint_size);
    // keep track of the longest row, if any are shorter don't include them as they're invalid
    // constraints
    let mut max_len = 0;
    let mut initial_pass = true;
    for r in &x.rows {
        let rl = r.len();
        // if the length changes and it's not the first pass clear out all the previous constraints
        if rl > max_len {
            if !initial_pass && rl < max_len {
                uv.clear();
            }
            max_len = r.len();
        }
        initial_pass = true;
        // if it's shorter than the max_length then we can skip it
        if rl < max_len {
            continue;
        }
        if let Some(v) = r.first() {
            let v = *v as i32;
            // we always calcluate u = 0 value for constraints, so skip it
            if v <= 0 {
                continue;
            }
            uv.push(v);
        }
    }
    // this is the actual constraint size
    let constraint_size = uv.len();
    // if we are calculating velocity we'll have 2 times the rows of the constraint size (add 1 because
    // we also have to add a col for the answer for the system of equations) otherwise we don't
    // care about the velocity
    let matrix_dim = (constraint_size + 1) * if velocity { 2 } else { 1 };
    // compute the intial constraints for position and velocity
    // these are always the same beside the length
    let mut constraint_pos = vec![vec![1.0]];
    constraint_pos[0].append(&mut vec![0.0; matrix_dim - 1]);
    let mut constraint_vel = vec![vec![0.0, 1.0]];
    constraint_vel[0].append(&mut vec![0.0; matrix_dim - 2]);
    // we always compute u = 0 so we previously filtered out u <= 0, so this starts from u1..
    for u in uv {
        let mut pos_row = Vec::with_capacity(matrix_dim);
        let mut vel_row = Vec::with_capacity(matrix_dim);
        for col in 0..(matrix_dim as i32) {
            // vel is derivative of pos
            pos_row.push(u.pow(col as u32) as f32);
            vel_row.push((col * (u.pow((col - 1).try_into().unwrap_or(0)))) as f32);
        }
        constraint_pos.push(pos_row);
        constraint_vel.push(vel_row);
    }
    // unfortunately have to clone here, otherwise it would mutate the same matrix when row
    // reducing which is not what we want
    let mut res = row_reduce(
        &mut constraint_pos.clone(),
        &mut constraint_vel.clone(),
        &x.rows,
        velocity,
        "matrix_x",
    );
    let mut y = row_reduce(
        &mut constraint_pos,
        &mut constraint_vel,
        &y.rows,
        velocity,
        "matrix_y",
    );
    res.append(&mut y);
    res
}

type M = Vec<Vec<f32>>;

#[inline]
fn scale(row: &mut [f32], scale: f32) {
    for v in row.iter_mut() {
        *v *= scale;
    }
}

// to reduce the need of clone on certain rows we can do a little unsafe :)
#[inline]
fn add(dst: *mut f32, src: *const f32, value: f32, dst_len: usize) {
    unsafe {
        for i in 0..dst_len {
            *dst.add(i) += *src.add(i) * value;
        }
    }
}

// find the pivot point, if there is none then we the matrix is not useful to us and should just
// crash
#[inline]
fn pivot(m: &M, row: usize) -> usize {
    for (i, v) in m[row].iter().enumerate() {
        if v != &0.0 {
            return i;
        }
    }
    panic!("no pivot")
}

#[inline]
fn row_of_col(m: &M, col: usize) -> Option<usize> {
    let mut rows = vec![];
    for i in 0..m.len() - 1 {
        if pivot(m, i) == col {
            rows.push(i);
        }
    }
    rows.first().copied()
}

fn row_reduce(pos: &mut M, vel: &mut M, direction: &M, velocity: bool, dir_name: &str) -> Vec<f32> {
    let max_len = direction.len();
    // add constraint columns/answers for system of equations
    for (i, v) in pos.iter_mut().enumerate() {
        if i >= max_len {
            break;
        }
        v.push(direction[i][1]);
    }
    // if the velocity is added then we add those rows, since you don't always have to put velocity
    // we gotta check if it's in bounds to prevent panic
    if velocity {
        for (i, v) in vel.iter_mut().enumerate() {
            if i >= max_len || direction[i].len() < 3 {
                break;
            }
            v.push(direction[i][2]);
        }
        pos.append(vel);
    }
    // grab dom element and display matrix if it actually exists
    for c in 0..pos[0].len() - 1 {
        let pivot = pivot(pos, c);
        let scale_value = pos[c][pivot];
        scale(&mut pos[c], 1.0 / scale_value);
        for r in 0..pos.len() {
            if r == c {
                continue;
            }
            let add_value = pos[r][pivot];
            // fancy pointer stuff to prevent cloning row c when adding, we aren't accessing the
            // same Vec in memory because of the previous continue statement, so it's safe (enough)
            let len = pos[r].len();
            let pos_c = pos[c].as_ptr();
            // add the negative
            add(pos[r].as_mut_ptr(), pos_c, -add_value, len);
        }
    }
    // go back and forth with row swaping
    let mut row_pos = 0;
    for c in 0..pos[0].len() - 1 {
        let Some(v) = row_of_col(pos, c) else {
            continue;
        };
        pos.swap(v, row_pos);
        row_pos += 1;
    }
    // return the last values of the rows as they're the answer to the system of equations
    pos.iter().filter_map(|x| x.last()).copied().collect()
}

struct Csv<'a, T> {
    pub columns: Vec<&'a str>,
    pub rows: Vec<Vec<T>>,
}

// this is terrible, but basically take the poorly done csv file and attempt to parse it, it
// expects that you have "(x, y)" for Vec and Point coords (because csv requires that values with a
// coma be wrapped in quotes), this jankily takes care of that by spliting at ," for every row
// except the first (since that is col name)
impl<'a> Csv<'a, (f32, f32)> {
    pub fn from_str(data: &'a str) -> Result<Self, Error> {
        let mut columns: Vec<&str> = vec![];
        let mut rows: Vec<Vec<(f32, f32)>> = vec![];
        for (i, line) in data.lines().enumerate() {
            // first line take out the col names
            if i == 0 {
                columns = line.split(',').collect();
                continue;
            }
            let line: Vec<(f32, f32)> = line
                .replace(",,", ", ,")
                .split(",\"")
                .enumerate()
                .map(|(i, x)| {
                    let x = x.replace('"', "");
                    if i == 0 {
                        (
                            x.parse().expect("failed to parse"),
                            x.parse().expect("failed to parse"),
                        )
                    } else {
                        let x = x.replace(['(', ')'], "").replace(',', " ");
                        let vals: Vec<&str> = x.split_whitespace().collect();
                        if vals.len() != 2 {
                            panic!("value pair is more than a pair");
                        }
                        let vals: Result<Vec<f32>, _> =
                            vals.into_iter().map(|x| x.parse()).collect();
                        let Ok(vals) = vals else {
                            panic!("failed to parse f32");
                        };
                        (vals[0], vals[1])
                    }
                })
                .collect();
            rows.push(line);
        }
        Ok(Self { columns, rows })
    }

    #[inline]
    pub fn constraint_size(&self) -> Option<usize> {
        Some(self.rows.len() - 1)
    }

    pub fn split(self) -> (Csv<'a, f32>, Csv<'a, f32>) {
        let clen = self.columns.len();
        let rlen = self.rows.len();
        // we don't need the cols so we can just omit them
        let mut x = Csv {
            columns: vec![],
            rows: vec![Vec::with_capacity(clen); rlen],
        };
        let mut y = Csv {
            columns: vec![],
            rows: vec![Vec::with_capacity(clen); rlen],
        };
        for (i, row) in self.rows.iter().enumerate() {
            for value in row {
                x.rows[i].push(value.0);
                y.rows[i].push(value.1);
            }
        }
        (x, y)
    }
}

pub struct Constraint {
    constraint: usize,
    position: (f32, f32),
    velocity: (f32, f32),
}

impl Constraint {
    fn from_values(values: Vec<(f32, f32, f32, f32)>) -> Vec<Self> {
        values.into_iter().enumerate().map(|(i, c)| Self {
            constraint: i,
            position: (c.0, c.1),
            velocity: (c.2, c.3),
        }).collect()
    }
}
