import {Robot} from './Robot';
import {Goal} from './Goal';
import {Results} from './Results';
import {Canvas} from './Canvas';
import { Controller } from './Controller';

export class Runner {
	constructor() {
		window.canvas = new Canvas();
		window.results = new Results();

		this.restart();
	}

	tick() {
		this.controller.step(window.robot);
		window.canvas.render();
	}

	getCurrentRobot() {
		return window.robot;
	}

	resetContents() {
		window.results.clear();
	};

	/* --------------------------------------------------- */
	/*         end of command functions
	 /* --------------------------------------------------- */
	printErrors(msg) {
		window.results.renderErrors(msg);
	}

	restart() {
		const robot = new Robot();
		const goal = new Goal(robot.minReachableRadius(), robot.maxReachableRadius());
		this.controller = new Controller(goal);

		window.robot = robot;
		window.goal = goal;
	}

};
