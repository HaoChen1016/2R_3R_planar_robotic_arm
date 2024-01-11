export class Results {
	constructor() {
		this.reportMessageEle = document.getElementById("results");
	}

	render() {
		var currentRobot = window.gherkin.getCurrentRobot();
		this.reportMessageEle.innerHTML =
			`<span> Theta 0: ${currentRobot.theta_0} </span>` +
			`<span> Theta 1: ${currentRobot.theta_1} </span>`;
	}

	renderErrors(msg) {
		this.reportMessageEle.innerHTML = `<span id="error"> ${msg} </span>`;
	}

	clear() {
		this.reportMessageEle.innerHTML = '';
	}
}
