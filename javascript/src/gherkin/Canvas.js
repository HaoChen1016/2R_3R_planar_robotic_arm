import {World} from './World';

export class Canvas {

	constructor() {
		// dimensions for App.js - how to set in only one place?
		this.canvasDimX = 580;
		this.canvasDimY = 580;

		this.world = new World(
			this.canvasDimX, this.canvasDimY,
			[this.canvasDimX/2, this.canvasDimY/2],
			[240, 240]
		)

		this.canvas = document.getElementById("canvas");
		this.context = this.canvas.getContext("2d");
	}

	render() {
		this.context.clearRect(0, 0, this.canvasDimX, this.canvasDimY);
		this.renderCanvas();
		this.renderGoal(window.gherkin.getCurrentRobot());
		this.renderRobot();

	}

	atGoal(robot, goal) {
		return (robot.x === goal.x && robot.y === goal.y);
	}

	renderCanvas() {
		this.context.strokeStyle = "#000";

		this.context.beginPath();
		this.context.rect(1, 1, this.canvasDimX-2, this.canvasDimY-2);

		this.context.stroke();
	}

	renderRobot() {
		let robot = window.gherkin.getCurrentRobot();
		let j0 = this.world.robotOrigin;
		let j1 = this.world.convertToDisplay(robot.joint_1_pos());
		let j2 = this.world.convertToDisplay(robot.joint_2_pos());

		this.context.beginPath()

		let jointRadius = 4;

		// Draw joint 0
		this.context.moveTo(j0[0] + jointRadius, j0[1]);
		this.context.arc(j0[0], j0[1], jointRadius, 0, 2 * Math.PI);
		// Draw link 1
		this.context.moveTo(j0[0], j0[1]);
		this.context.lineTo(j1[0], j1[1]);
		// Draw joint 1
		this.context.moveTo(j1[0] + jointRadius, j1[1]);
		this.context.arc(j1[0], j1[1], jointRadius, 0, 2 * Math.PI);
		// Draw link 2
		this.context.moveTo(j1[0], j1[1]);
		this.context.lineTo(j2[0], j2[1]);
		// Draw joint 2
		this.context.moveTo(j2[0] + jointRadius, j2[1]);
		this.context.arc(j2[0], j2[1], jointRadius, 0, 2 * Math.PI);

		this.context.fillStyle = robot.color;
		this.context.stroke();

		window.results.render();
	}

	renderGoal(robot) {
		var goal = window.goal;

		const [centerX, centerY] =
			this.world.convertToDisplay([goal.x, goal.y]);
		var radius = 6;

		var context = this.context;

		var path = new Path2D();
		path.arc(centerX, centerY, radius, 0, 2 * Math.PI, false);
		path.closePath();

		if (this.atGoal(robot, goal)) {
			context.fillStyle = "blue";
		}

		context.stroke(path);
		context.fill(path);

		context.fillStyle = "red";
	}
}
