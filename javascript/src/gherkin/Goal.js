import {getRandom} from '../utils.js';

export class Goal {

	constructor(minRadius, maxRadius) {
    // Ensure theta is not 0
    const theta = Math.random() * 2 * Math.PI;
    // Ensure point is reachable
    const r = getRandom(minRadius, maxRadius);

    this.x = r * Math.cos(theta);
    this.y = r * Math.sin(theta);


		console.log(`Goal positioned at ${this.x}, ${this.y}`);
	}

}
