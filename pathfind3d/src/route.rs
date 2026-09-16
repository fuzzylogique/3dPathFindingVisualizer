//! Route arithmetic: what a sequence of cells costs, whether it is legal,
//! and how to bridge two cells that are not neighbours.
//!
//! These back the "draw your own route and race the algorithm" mode. They
//! used to live in JavaScript, which meant the scoreboard was computed by a
//! different implementation than the one being scored against.

use std::collections::VecDeque;

use crate::domain::Domain;
use crate::solver::STEP_COST;
use crate::{T_OBST, T_VOID};

/// True weighted cost of a path: per step, the Euclidean base cost of the
/// (wrap-aware) move times the weight of the cell being entered.
pub fn path_cost(dom: &Domain, terrain: &[u8], path: &[u32]) -> f64 {
    let mut acc = 0.0;
    for w in path.windows(2) {
        let (ax, ay, az) = dom.coords(w[0]);
        let (bx, by, bz) = dom.coords(w[1]);
        let k = (dom.axis_delta(ax, bx, 0) + dom.axis_delta(ay, by, 1) + dom.axis_delta(az, bz, 2))
            as usize;
        if k >= STEP_COST.len() {
            return f64::NAN; // not a neighbour move; `validate_route` explains why
        }
        acc += STEP_COST[k] * terrain[w[1] as usize] as f64;
    }
    acc
}

/// `None` when the route is legal, otherwise the reason it is not: it must
/// begin at the start, end at the goal, stay on existing unblocked cells, and
/// take one legal neighbour step at a time.
pub fn validate_route(
    dom: &Domain,
    terrain: &[u8],
    path: &[u32],
    start: u32,
    goal: u32,
    diagonals: bool,
) -> Option<&'static str> {
    if path.is_empty() {
        return Some("empty route");
    }
    if path[0] != start {
        return Some("must begin at start");
    }
    if *path.last().unwrap() != goal {
        return Some("must reach the goal");
    }
    for (s, &cell) in path.iter().enumerate() {
        if cell as usize >= terrain.len() {
            return Some("route leaves the domain");
        }
        match terrain[cell as usize] {
            T_VOID => return Some("route leaves the domain"),
            T_OBST => return Some("route crosses an obstacle"),
            _ => {}
        }
        if s == 0 {
            continue;
        }
        let (ax, ay, az) = dom.coords(path[s - 1]);
        let (bx, by, bz) = dom.coords(cell);
        let (dx, dy, dz) = (
            dom.axis_delta(ax, bx, 0),
            dom.axis_delta(ay, by, 1),
            dom.axis_delta(az, bz, 2),
        );
        if dx > 1.0 || dy > 1.0 || dz > 1.0 {
            return Some("route jumps cells");
        }
        let k = dx + dy + dz;
        if k < 1.0 {
            return Some("route repeats a cell");
        }
        if !diagonals && k > 1.0 {
            return Some("diagonal step with diagonals off");
        }
    }
    None
}

/// Shortest hop-count leg between two cells, inclusive of both ends, or
/// `None` if no open route exists. Drawing a route in 3D would be miserable
/// if every cell had to be clicked, so distant clicks are bridged with this.
pub fn connect(
    dom: &Domain,
    terrain: &[u8],
    from: u32,
    to: u32,
    diagonals: bool,
) -> Option<Vec<u32>> {
    if from == to {
        return Some(vec![from]);
    }
    let total = dom.total();
    if !dom.exists_at(from) || !dom.exists_at(to) {
        return None;
    }
    if terrain[to as usize] == T_OBST || terrain[from as usize] == T_OBST {
        return None;
    }
    let mut parent = vec![u32::MAX; total];
    let mut seen = vec![false; total];
    let mut q = VecDeque::new();
    seen[from as usize] = true;
    q.push_back(from);
    while let Some(cur) = q.pop_front() {
        for dz in -1i32..=1 {
            for dy in -1i32..=1 {
                for dx in -1i32..=1 {
                    if dx == 0 && dy == 0 && dz == 0 {
                        continue;
                    }
                    if !diagonals && dx.abs() + dy.abs() + dz.abs() > 1 {
                        continue;
                    }
                    let Some(ni) = dom.neighbor(cur, dx, dy, dz) else {
                        continue;
                    };
                    let nu = ni as usize;
                    if seen[nu] || terrain[nu] == T_OBST || terrain[nu] == T_VOID {
                        continue;
                    }
                    seen[nu] = true;
                    parent[nu] = cur;
                    if ni == to {
                        let mut rev = vec![to];
                        let mut c = to;
                        while parent[c as usize] != u32::MAX {
                            c = parent[c as usize];
                            rev.push(c);
                        }
                        rev.reverse();
                        return Some(rev);
                    }
                    q.push_back(ni);
                }
            }
        }
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn adjacent_steps_pass_jumps_and_obstacles_fail() {
        let dom = Domain::raw(10, 10, 10, [false; 3]);
        let mut terrain = vec![1u8; dom.total()];
        let a = dom.idx(0, 0, 0);
        let b = dom.idx(1, 0, 0);
        let far = dom.idx(5, 5, 5);
        assert!(validate_route(&dom, &terrain, &[a, b], a, b, true).is_none());
        assert!(validate_route(&dom, &terrain, &[a, far], a, far, true).is_some());
        terrain[b as usize] = T_OBST;
        assert_eq!(
            validate_route(&dom, &terrain, &[a, b], a, b, true),
            Some("route crosses an obstacle")
        );
    }

    #[test]
    fn diagonal_step_is_rejected_when_diagonals_are_off() {
        let dom = Domain::raw(5, 5, 5, [false; 3]);
        let terrain = vec![1u8; dom.total()];
        let a = dom.idx(0, 0, 0);
        let b = dom.idx(1, 1, 0);
        assert!(validate_route(&dom, &terrain, &[a, b], a, b, true).is_none());
        assert_eq!(
            validate_route(&dom, &terrain, &[a, b], a, b, false),
            Some("diagonal step with diagonals off")
        );
    }

    #[test]
    fn connect_bridges_around_a_wall_and_gives_a_legal_route() {
        let dom = Domain::raw(7, 7, 1, [false; 3]);
        let mut terrain = vec![1u8; dom.total()];
        for y in 0..6 {
            terrain[dom.idx(3, y, 0) as usize] = T_OBST;
        }
        let a = dom.idx(0, 0, 0);
        let b = dom.idx(6, 0, 0);
        let leg = connect(&dom, &terrain, a, b, true).expect("a way around the wall exists");
        assert_eq!(leg[0], a);
        assert_eq!(*leg.last().unwrap(), b);
        assert!(validate_route(&dom, &terrain, &leg, a, b, true).is_none());
    }

    #[test]
    fn connect_reports_no_route_when_the_target_is_sealed_off() {
        let dom = Domain::raw(7, 7, 1, [false; 3]);
        let mut terrain = vec![1u8; dom.total()];
        for y in 0..7 {
            terrain[dom.idx(3, y, 0) as usize] = T_OBST;
        }
        let a = dom.idx(0, 0, 0);
        let b = dom.idx(6, 0, 0);
        assert!(connect(&dom, &terrain, a, b, true).is_none());
    }

    #[test]
    fn wrapped_axis_makes_opposite_faces_adjacent() {
        let dom = Domain::raw(9, 3, 3, [true, false, false]);
        let terrain = vec![1u8; dom.total()];
        let a = dom.idx(0, 1, 1);
        let b = dom.idx(8, 1, 1);
        assert!(validate_route(&dom, &terrain, &[a, b], a, b, false).is_none());
        assert!((path_cost(&dom, &terrain, &[a, b]) - 1.0).abs() < 1e-12);
    }
}
