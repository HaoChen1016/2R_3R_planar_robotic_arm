import {Robot} from './Robot';

export class Controller {
	constructor(goal) {
		this.goal = goal;

		[this.goal_theta_0, this.goal_theta_1] =
			Robot.inverse(this.goal.x, this.goal.y);
	}

	step(robot) {
		// Simple P controller
		const theta_0_error = this.goal_theta_0 - robot.theta_0;
		const theta_1_error = this.goal_theta_1 - robot.theta_1;

		robot.theta_0 += theta_0_error / 10;
		robot.theta_1 += theta_1_error / 10;

		return robot;
	}
}
