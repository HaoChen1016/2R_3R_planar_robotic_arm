export function getRandom(min, max) {
	min = Math.ceil(min);
	max = Math.floor(max);
	return Math.random() * (max - min + 1) + min;
}

export function assert(condition, message) {
    if (!condition) {
        throw message || "Assertion failed";
    }
}