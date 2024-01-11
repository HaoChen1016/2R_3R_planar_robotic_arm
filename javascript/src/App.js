import './App.css';
import {Runner} from './gherkin/Runner.js';
import { useEffect } from 'react';


function App() {

  useEffect(() => {

    const runner = new Runner();
    window.gherkin = runner;

    var gameLoop = function(){
      runner.tick();

      window.requestAnimationFrame(gameLoop);
    }
    window.requestAnimationFrame(gameLoop);
  })
  return (
    <div className="App">
      <h1>Gherkin Challenge</h1>
      <canvas id="canvas" width="580" height="580"></canvas>
      <div id="results"></div>
    </div>
  );
}

export default App;
