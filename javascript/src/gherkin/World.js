export class World {
	constructor(
			width,
			height,
			robotOrigin,
			goal,
	) {
		this.width = width;
		this.height = height;
		this.robotOrigin = robotOrigin;
		this.goal = goal;
	}

	convertToDisplay(point) {
		// Convert a point from the robot coordinate system to the display coordinate system
		const [robot_x, robot_y] = point;
		const [offset_x, offset_y] = this.robotOrigin;

		return [offset_x + robot_x, offset_y - robot_y];
	}

}
