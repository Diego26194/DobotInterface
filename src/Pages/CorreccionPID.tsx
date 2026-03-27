import React, { useState, useEffect } from "react";
import {initRos, pubTopic, subTopic, unsubTopic } from '../Services/RosService2';
import {  Stack } from "@mui/material";

function modificarPID( coord:number[]) {
  console.log("Enviar PID:", coord);
  const msg = {
    data: coord
  };

  //console.log(" Enviando a /pos_dy: correr angulos", coord);

  // Publica en ROS
  pubTopic('mod_pid', msg);
}

const CorreccionPID = () => {
  // Estados para cada grupo de inputs (arrays de enteros)
  const [articulacion1, setarticulacion1] = useState([640, 0, 4000, 0, 0]);
  const [articulacion2, setarticulacion2] = useState([640, 0, 4000, 0, 0]);
  const [articulacion3, setarticulacion3] = useState([640, 0, 4000, 0, 0]);
  const indicadores = ["P", "I", "D", "TT1", "TT2"];
  
  
useEffect(() => {
  // inicializa ROS solo una vez cuando el componente se monta
  initRos();
}, [])


  // Handler genérico para inputs
  const handleInputChange = (
    index: number,
    value: string,
    setState: React.Dispatch<React.SetStateAction<number[]>>,
    currentState: number[]
    ) => {
    const newValues = [...currentState];
    // Convertir a entero, si no es válido -> 0
    newValues[index] = parseInt(value, 10) || 0;
    setState(newValues);
  };

  // Funciones vacías para los botones
  const cambiarPID1 = () => {
    modificarPID([1, ...articulacion1]);
  };

  const cambiarPID2 = () => {
    modificarPID([2, ...articulacion2]);
  };

  const cambiarPID3 = () => {
    modificarPID([3, ...articulacion3]);
  };

   return (
    
    
    
    <div style={{ padding: "20px" }}>
      <Stack direction="row" spacing={8}>
          {/* Sección 1 */}
        <div>
          <h2>Articulacion 1</h2>
          {articulacion1.map((val, i) => (
            <div key={i} style={{ margin: "5px" }}>
              <input
                type="number"
                value={val}
                min={-1}
                max={4095}
                onChange={(e) =>
                  handleInputChange(i, e.target.value, setarticulacion1, articulacion1)
                }
                style={{ width: "80px", marginRight: "10px" }}
              />
              <span>{indicadores[i]}</span>
              
            </div>
          ))}
          <br />
          <button onClick={cambiarPID1}>Cambiar PID 1</button>
        </div>

          {/* Sección 2 */}
        <div>
          <h2>Articulacion 2</h2>
          {articulacion2.map((val, i) => (
            <div key={i} style={{ margin: "5px" }}>
              <input
                type="number"
                value={val}
                min={-1}
                max={4095}
                onChange={(e) =>
                  handleInputChange(i, e.target.value, setarticulacion2, articulacion2)
                }
                style={{ width: "80px", marginRight: "10px" }}
              />              
              <span>{indicadores[i]}</span>
            </div>
          ))}
          <br />
          <button onClick={cambiarPID2}>Cambiar PID 2</button>
        </div>

          {/* Sección 3 */}
        <div>
          <h2>Articulacion 3</h2>
          {articulacion3.map((val, i) => (
            <div key={i} style={{ margin: "5px" }}>
              <input
                type="number"
                value={val}
                min={-1}
                max={4095}
                onChange={(e) => handleInputChange(i, e.target.value, setarticulacion3, articulacion3)}
                style={{ width: "80px", marginRight: "10px" }}
              />
              <span>{indicadores[i]}</span>
            </div>
          ))}
          <br />
          <button onClick={cambiarPID3}>Cambiar PID 3</button>
        </div>

        </Stack>
    </div>
  );
};

export default CorreccionPID;
