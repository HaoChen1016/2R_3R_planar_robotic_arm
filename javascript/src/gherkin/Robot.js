import {assert} from '../utils.js';

const JOINT_LIMITS = [-6.28, 6.28];
const MAX_VELOCITY = 15;
const MAX_ACCELERATION = 50;
const DT = 0.033;


export class Robot {
	static LINK_1 = 75; // pixels
	static LINK_2 = 50; // pixels

	constructor() {
		this._theta_0 = 0; // radians
		this._theta_1 = 0; // radians

		// These keep growing infinitely
		this.all_theta_0 = [];
		this.all_theta_1 = [];
	}

	// Getters/Setters
	get theta_0() {
		return this._theta_0;
	}

	set theta_0(value) {
		this.all_theta_0.push(value)
		this._theta_0 = value
		// Check limits
		assert(this.check_angle_limits(value), `Joint 0 value ${value} exceeds joint limits`);
		assert(this.max_velocity(this.all_theta_0) < MAX_VELOCITY,
		`Joint 0 Velocity ${this.max_velocity(this.all_theta_0)} exceeds velocity limit ${MAX_VELOCITY}`);
		assert(this.max_acceleration(this.all_theta_0) < MAX_ACCELERATION,
		`Joint 0 Accel ${this.max_acceleration(this.all_theta_0)} exceeds acceleration limit ${MAX_ACCELERATION}`);
	}

	get theta_1() {
		return this._theta_1
	}

	set theta_1(value) {
		this.all_theta_1.push(value)
		this._theta_1 = value
		assert(this.check_angle_limits(value), `Joint 1 value ${value} exceeds joint limits`);
		assert(this.max_velocity(this.all_theta_1) < MAX_VELOCITY,
			`Joint 1 Velocity ${this.max_velocity(this.all_theta_1)} exceeds velocity limit ${MAX_VELOCITY}`);
		assert(this.max_acceleration(this.all_theta_1) < MAX_ACCELERATION,
			`Joint 1 Accel ${this.max_acceleration(this.all_theta_1)} exceeds acceleration limit ${MAX_ACCELERATION}`);
	}

	// Kinematics

	joint_1_pos() {
		// Compute the x, y position of joint 1
		return [Robot.LINK_1 * Math.cos(this.theta_0), Robot.LINK_1 * Math.sin(this.theta_0)];
	}

	joint_2_pos() {
		// Compute the x, y position of joint 2
		return this.forward(this.theta_0, this.theta_1);
	}

	forward(theta_0, theta_1) {
		// Compute the x, y position of the end of the links from the joint angles
		const x = Robot.LINK_1 * Math.cos(theta_0) + Robot.LINK_2 * Math.cos(theta_0 + theta_1);
		const y = Robot.LINK_1 * Math.sin(theta_0) + Robot.LINK_2 * Math.sin(theta_0 + theta_1);

		return [x, y];

	}

	static inverse(x, y) {
		// Compute the joint angles from the position of the end of the links

		const theta_1 = Math.acos((x ** 2 + y ** 2 - Robot.LINK_1 ** 2 - Robot.LINK_2 ** 2)
												/ (2 * Robot.LINK_1 * Robot.LINK_2));
		const theta_0 = Math.atan2(y, x) -
		Math.atan((Robot.LINK_2 * Math.sin(theta_1)) /
									(Robot.LINK_1 + Robot.LINK_2 * Math.cos(theta_1)));

		return [theta_0, theta_1];
	}

	check_angle_limits(theta) {
		return JOINT_LIMITS[0] < theta < JOINT_LIMITS[1];
	}

	 max_velocity(all_theta) {
		return aMax(aAbs(aDiff(all_theta).map((i) => i / DT)), 0.);
	 }

	max_acceleration(all_theta) {
		return aMax(aAbs(aDiff(aDiff(all_theta)).map((i) => i / DT).map((i) => i / DT)), 0.);
	}

	minReachableRadius() {
		return Math.max(Robot.LINK_1 - Robot.LINK_2, 0)
	}

	maxReachableRadius() {
		return Robot.LINK_1 + Robot.LINK_2
	}

}

function aAbs(array) {
	// return the absolute value of all array elements
	return array.map(Math.abs);
}

function aDiff(array) {
	// Like numpy.diff. A unit test would be nice.
	return array.reduce((result, e, index, array) => {
		if (index+1 < array.length) {
			result.push(array[index+1] - e);
		}
		return result;
	}, []);
}

function aMax(array, min) {
	const max = array.reduce(function(a, b) {
			return Math.max(a, b);
	}, min);
	return max;
}
