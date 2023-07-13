// adapted from python code here -> https://github.com/fgabbert/dubins_py/blob/master/dubins_py.py

class TurnType {
  static LSL = 1;
  static LSR = 2;
  static RSL = 3;
  static RSR = 4;
  static RLR = 5;
  static LRL = 6;

  static fromNumberToTurnType(n: number) {
    switch (n) {
      case 1:
        return TurnType.LSL;
      case 2:
        return TurnType.LSR;
      case 3:
        return TurnType.RSL;
      case 4:
        return TurnType.RSR;
      case 5:
        return TurnType.RLR;
      case 6:
        return TurnType.LRL;
      default:
        return TurnType.LSL;
    }
  }

  static fromTurnTypeToNumber(t: TurnType) {
    switch (t) {
      case TurnType.LSL:
        return 1;
      case TurnType.LSR:
        return 2;
      case TurnType.RSL:
        return 3;
      case TurnType.RSR:
        return 4;
      case TurnType.RLR:
        return 5;
      case TurnType.LRL:
        return 6;
      default:
        return 1;
    }
  }
}

export class Waypoint {
  x: number;
  y: number;
  psi: number;

  constructor(x: number, y: number, psi: number) {
    this.x = x;
    this.y = y;
    this.psi = psi;
  }

  toString() {
    return "x: " + this.x + ", y: " + this.y + ", psi: " + this.psi;
  }
}

class Param {
  p_init: Waypoint;
  seg_final: [number, number, number];
  turn_radius: number;
  type: TurnType;

  constructor(
    p_init: Waypoint,
    seg_final: [number, number, number],
    turn_radius: number
  ) {
    this.p_init = p_init;
    this.seg_final = seg_final;
    this.turn_radius = turn_radius;
    this.type = 0;
  }
}

class Trajectory {
  x: number[];
  y: number[];

  constructor(x: number[], y: number[]) {
    this.x = x;
    this.y = y;
  }
}

function wrapTo360(angle: number) {
  let posIn = angle > 0;
  angle = pythonModulo(angle, 360);
  if (angle === 0 && posIn) {
    angle = 360;
  }
  return angle;
}

function wrapTo180(angle: number) {
  let q = angle < -180 || 180 < angle;
  if (q) {
    angle = wrapTo360(angle + 180) - 180;
  }
  return angle;
}

function headingToStandard(hdg: number) {
  // Convert NED heading to standard unit cirlce...degrees only for now (Im lazy)
  let thet = wrapTo360(90 - wrapTo180(hdg));
  return thet;
}

// rest of the functions go here

export function main() {
  // User's waypoints: [x, y, heading (degrees)]
  let pt1 = new Waypoint(0, 0, 0);
  let pt2 = new Waypoint(6000, 7000, 260);
  //   let pt3 = new Waypoint(1000, 15000, 180);
  //   let pt4 = new Waypoint(0, 0, 270);
  let Wptz = [pt1, pt2];
  // Run the code
  let i = 0;
  while (i < Wptz.length - 1) {
    let param = calcDubinsPath(Wptz[i], Wptz[i + 1], 90, 20);
    console.log({ param });
    const path = dubinsTraj(param, 20);
    console.log({ path });
    console.log("path length: " + path.length);
    i++;
  }
}

function dubinsLSL(
  alpha: number,
  beta: number,
  d: number
): [number, number, number] {
  let tmp0: number = d + Math.sin(alpha) - Math.sin(beta);
  let tmp1: number = Math.atan2(Math.cos(beta) - Math.cos(alpha), tmp0);
  let p_squared: number =
    2 +
    d * d -
    2 * Math.cos(alpha - beta) +
    2 * d * (Math.sin(alpha) - Math.sin(beta));
  let p: number;
  let q: number;
  let t: number;
  if (p_squared < 0) {
    console.log("No LSL Path");
    p = -1;
    q = -1;
    t = -1;
  } else {
    // t = (tmp1 - alpha) % (2 * Math.PI);
    t = pythonModulo(tmp1 - alpha, 2 * Math.PI);
    p = Math.sqrt(p_squared);
    // q = (beta - tmp1) % (2 * Math.PI);
    q = pythonModulo(beta - tmp1, 2 * Math.PI);
  }
  console.log({ tmp1, alpha });
  return [t, p, q];
}

function dubinsRSR(
  alpha: number,
  beta: number,
  d: number
): [number, number, number] {
  let tmp0: number = d - Math.sin(alpha) + Math.sin(beta);
  let tmp1: number = Math.atan2(Math.cos(alpha) - Math.cos(beta), tmp0);
  let p_squared: number =
    2 +
    d * d -
    2 * Math.cos(alpha - beta) +
    2 * d * (Math.sin(beta) - Math.sin(alpha));
  let p: number;
  let q: number;
  let t: number;
  if (p_squared < 0) {
    console.log("No RSR Path");
    p = -1;
    q = -1;
    t = -1;
  } else {
    // t = (alpha - tmp1) % (2 * Math.PI);
    t = pythonModulo(alpha - tmp1, 2 * Math.PI);
    p = Math.sqrt(p_squared);
    // q = (tmp1 - beta) % (2 * Math.PI);
    q = pythonModulo(tmp1 - beta, 2 * Math.PI);
  }
  return [t, p, q];
}

function dubinsRSL(
  alpha: number,
  beta: number,
  d: number
): [number, number, number] {
  let tmp0: number = d - Math.sin(alpha) - Math.sin(beta);
  let p_squared: number =
    -2 +
    d * d +
    2 * Math.cos(alpha - beta) -
    2 * d * (Math.sin(alpha) + Math.sin(beta));
  let p: number;
  let q: number;
  let t: number;
  if (p_squared < 0) {
    console.log("No RSL Path");
    p = -1;
    q = -1;
    t = -1;
  } else {
    p = Math.sqrt(p_squared);
    let tmp2: number =
      Math.atan2(Math.cos(alpha) + Math.cos(beta), tmp0) - Math.atan2(2, p);
    // t = (alpha - tmp2) % (2 * Math.PI);
    t = pythonModulo(alpha - tmp2, 2 * Math.PI);
    // q = (beta - tmp2) % (2 * Math.PI);
    q = pythonModulo(beta - tmp2, 2 * Math.PI);
  }
  return [t, p, q];
}

function dubinsLSR(
  alpha: number,
  beta: number,
  d: number
): [number, number, number] {
  let tmp0: number = d + Math.sin(alpha) + Math.sin(beta);
  let p_squared: number =
    -2 +
    d * d +
    2 * Math.cos(alpha - beta) +
    2 * d * (Math.sin(alpha) + Math.sin(beta));
  let p: number;
  let q: number;
  let t: number;
  if (p_squared < 0) {
    console.log("No LSR Path");
    p = -1;
    q = -1;
    t = -1;
  } else {
    p = Math.sqrt(p_squared);
    let tmp2: number =
      Math.atan2(-1 * Math.cos(alpha) - Math.cos(beta), tmp0) -
      Math.atan2(-2, p);
    // t = (tmp2 - alpha) % (2 * Math.PI);
    t = pythonModulo(tmp2 - alpha, 2 * Math.PI);
    // q = (tmp2 - beta) % (2 * Math.PI);
    q = pythonModulo(tmp2 - beta, 2 * Math.PI);
  }
  return [t, p, q];
}

function dubinsRLR(
  alpha: number,
  beta: number,
  d: number
): [number, number, number] {
  let tmp_rlr: number =
    (6 -
      d * d +
      2 * Math.cos(alpha - beta) +
      2 * d * (Math.sin(alpha) - Math.sin(beta))) /
    8;
  let p: number;
  let q: number;
  let t: number;
  if (Math.abs(tmp_rlr) > 1) {
    console.log("No RLR Path");
    p = -1;
    q = -1;
    t = -1;
  } else {
    // p = (2 * Math.PI - Math.acos(tmp_rlr)) % (2 * Math.PI);
    p = pythonModulo(2 * Math.PI - Math.acos(tmp_rlr), 2 * Math.PI);
    // t =
    //   (alpha -
    //     Math.atan2(
    //       Math.cos(alpha) - Math.cos(beta),
    //       d - Math.sin(alpha) + Math.sin(beta)
    //     ) +
    //     ((p / 2) % (2 * Math.PI))) %
    //   (2 * Math.PI);
    t = pythonModulo(
      alpha -
        Math.atan2(
          Math.cos(alpha) - Math.cos(beta),
          d - Math.sin(alpha) + Math.sin(beta)
        ) +
        pythonModulo(p / 2, 2 * Math.PI),
      2 * Math.PI
    );
    // q = (alpha - beta - t + (p % (2 * Math.PI))) % (2 * Math.PI);
    q = pythonModulo(
      alpha - beta - t + pythonModulo(p, 2 * Math.PI),
      2 * Math.PI
    );
  }
  return [t, p, q];
}

function dubinsLRL(
  alpha: number,
  beta: number,
  d: number
): [number, number, number] {
  let tmp_lrl: number =
    (6 -
      d * d +
      2 * Math.cos(alpha - beta) +
      2 * d * (-1 * Math.sin(alpha) + Math.sin(beta))) /
    8;
  let p: number;
  let q: number;
  let t: number;
  if (Math.abs(tmp_lrl) > 1) {
    console.log("No LRL Path");
    p = -1;
    q = -1;
    t = -1;
  } else {
    // p = (2 * Math.PI - Math.acos(tmp_lrl)) % (2 * Math.PI);
    p = pythonModulo(2 * Math.PI - Math.acos(tmp_lrl), 2 * Math.PI);
    // t =
    //   (-1 * alpha -
    //     Math.atan2(
    //       Math.cos(alpha) - Math.cos(beta),
    //       d + Math.sin(alpha) - Math.sin(beta)
    //     ) +
    //     p / 2) %
    //   (2 * Math.PI);
    t = pythonModulo(
      -1 * alpha -
        Math.atan2(
          Math.cos(alpha) - Math.cos(beta),
          d + Math.sin(alpha) - Math.sin(beta)
        ) +
        p / 2,
      2 * Math.PI
    );

    // q =
    //   ((beta % (2 * Math.PI)) - alpha - t + (p % (2 * Math.PI))) %
    //   (2 * Math.PI);
    q = pythonModulo(
      pythonModulo(beta, 2 * Math.PI) -
        alpha -
        t +
        pythonModulo(p, 2 * Math.PI),
      2 * Math.PI
    );
  }
  return [t, p, q];
}

export function calcDubinsPath(
  waypoint1: Waypoint,
  waypoint2: Waypoint,
  vel: number,
  phi_lim: number,
  turnRadius?: number
): Param {
  console.log({ waypoint1, waypoint2, vel, phi_lim, turnRadius });
  // Calculate a dubins path between two waypoints
  let param: Param = new Param(waypoint1, [0, 0, 0], 0);
  let tz: number[] = [0, 0, 0, 0, 0, 0];
  let pz: number[] = [0, 0, 0, 0, 0, 0];
  let qz: number[] = [0, 0, 0, 0, 0, 0];
  param.seg_final = [0, 0, 0];
  // Convert the headings from NED to standard unit cirlce, and then to radians
  let psi1: number = (headingToStandard(waypoint1.psi) * Math.PI) / 180;
  let psi2: number = (headingToStandard(waypoint2.psi) * Math.PI) / 180;

  // Do math
  param.turn_radius = turnRadius
    ? turnRadius
    : (vel * vel) / (9.8 * Math.tan((phi_lim * Math.PI) / 180));
  let dx: number = waypoint2.x - waypoint1.x;
  let dy: number = waypoint2.y - waypoint1.y;
  let D: number = Math.sqrt(dx * dx + dy * dy);
  let d: number = D / param.turn_radius; // Normalize by turn radius...makes length calculation easier down the road.

  // Angles defined in the paper
  let theta: number = pythonModulo(Math.atan2(dy, dx), 2 * Math.PI);
  let alpha: number = pythonModulo(psi1 - theta, 2 * Math.PI);
  let beta: number = pythonModulo(psi2 - theta, 2 * Math.PI);
  let best_word: number = -1;
  let best_cost: number = -1;

  // Calculate all dubin's paths between points
  [tz[0], pz[0], qz[0]] = dubinsLSL(alpha, beta, d);
  [tz[1], pz[1], qz[1]] = dubinsLSR(alpha, beta, d);
  [tz[2], pz[2], qz[2]] = dubinsRSL(alpha, beta, d);
  [tz[3], pz[3], qz[3]] = dubinsRSR(alpha, beta, d);
  [tz[4], pz[4], qz[4]] = dubinsRLR(alpha, beta, d);
  [tz[5], pz[5], qz[5]] = dubinsLRL(alpha, beta, d);

  console.log({ tz });
  console.log({ pz });
  console.log({ qz });

  // Now, pick the one with the lowest cost
  for (let x = 0; x < 6; x++) {
    if (tz[x] != -1) {
      let cost: number = tz[x] + pz[x] + qz[x];
      if (cost < best_cost || best_cost == -1) {
        best_word = x + 1;
        best_cost = cost;
        param.seg_final = [tz[x], pz[x], qz[x]];
      }
    }
  }

  param.type = TurnType.fromNumberToTurnType(best_word);
  return param;
}

export function dubinsTraj(param: Param, step: number): number[][] {
  // Build the trajectory from the lowest-cost path
  let x: number = 0;
  let i: number = 0;
  let length: number =
    (param.seg_final[0] + param.seg_final[1] + param.seg_final[2]) *
    param.turn_radius;
  length = Math.floor(length / step);
  let path: number[][] = Array(length).fill([-1, -1, -1]);

  while (x < length) {
    path[i] = dubinsPath(param, x);
    x += step;
    i += 1;
  }
  return path;
}

export function dubinsPath(param: Param, t: number): number[] {
  // Helper function for curve generation
  let tprime: number = t / param.turn_radius;
  let p_init: number[] = [
    0,
    0,
    headingToStandard(param.p_init.psi) * (Math.PI / 180),
  ];
  //
  let L_SEG: number = 1;
  let S_SEG: number = 2;
  let R_SEG: number = 3;
  let DIRDATA: number[][] = [
    [L_SEG, S_SEG, L_SEG],
    [L_SEG, S_SEG, R_SEG],
    [R_SEG, S_SEG, L_SEG],
    [R_SEG, S_SEG, R_SEG],
    [R_SEG, L_SEG, R_SEG],
    [L_SEG, R_SEG, L_SEG],
  ];
  //
  let types: number[] = DIRDATA[TurnType.fromTurnTypeToNumber(param.type) - 1];
  let param1: number = param.seg_final[0];
  let param2: number = param.seg_final[1];
  let mid_pt1: number[] = dubinsSegment(param1, p_init, types[0]);
  let mid_pt2: number[] = dubinsSegment(param2, mid_pt1, types[1]);

  if (tprime < param1) {
    let end_pt: number[] = dubinsSegment(tprime, p_init, types[0]);
    return [
      end_pt[0] * param.turn_radius + param.p_init.x,
      end_pt[1] * param.turn_radius + param.p_init.y,
      pythonModulo(end_pt[2], 2 * Math.PI),
    ];
  } else if (tprime < param1 + param2) {
    let end_pt: number[] = dubinsSegment(tprime - param1, mid_pt1, types[1]);
    return [
      end_pt[0] * param.turn_radius + param.p_init.x,
      end_pt[1] * param.turn_radius + param.p_init.y,
      pythonModulo(end_pt[2], 2 * Math.PI),
    ];
  } else {
    let end_pt: number[] = dubinsSegment(
      tprime - param1 - param2,
      mid_pt2,
      types[2]
    );
    return [
      end_pt[0] * param.turn_radius + param.p_init.x,
      end_pt[1] * param.turn_radius + param.p_init.y,
      pythonModulo(end_pt[2], 2 * Math.PI),
    ];
  }
}

export function dubinsSegment(
  seg_param: number,
  seg_init: number[],
  seg_type: number
): number[] {
  // Helper function for curve generation
  let L_SEG: number = 1;
  let S_SEG: number = 2;
  let R_SEG: number = 3;
  let seg_end: number[] = [0, 0, 0];
  if (seg_type == L_SEG) {
    seg_end[0] =
      seg_init[0] + Math.sin(seg_init[2] + seg_param) - Math.sin(seg_init[2]);
    seg_end[1] =
      seg_init[1] - Math.cos(seg_init[2] + seg_param) + Math.cos(seg_init[2]);
    seg_end[2] = seg_init[2] + seg_param;
  } else if (seg_type == R_SEG) {
    seg_end[0] =
      seg_init[0] - Math.sin(seg_init[2] - seg_param) + Math.sin(seg_init[2]);
    seg_end[1] =
      seg_init[1] + Math.cos(seg_init[2] - seg_param) - Math.cos(seg_init[2]);
    seg_end[2] = seg_init[2] - seg_param;
  } else if (seg_type == S_SEG) {
    seg_end[0] = seg_init[0] + Math.cos(seg_init[2]) * seg_param;
    seg_end[1] = seg_init[1] + Math.sin(seg_init[2]) * seg_param;
    seg_end[2] = seg_init[2];
  }

  return seg_end;
}

const pythonModulo = (n: number, m: number) => {
  return ((n % m) + m) % m;
};
