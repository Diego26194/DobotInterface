import React, { useState, useEffect } from "react";
import {initRos, pubTopic, subTopic, unsubTopic } from '../Services/RosService2';
import {  Stack } from "@mui/material";

function mandarCordReal( coord:number[]) {
  const msg = {
    data: coord
  };

  //console.log(" Enviando a /pos_dy: correr angulos", coord);

  // Publica en ROS
  pubTopic('pos_dy', msg);
}

function mandarPunto( coord:number[]) {
  const msg = {
    data: coord
  };

  console.log(" Enviando a /pos_dy: correr angulos", coord);

  // Publica en ROS
  pubTopic('p_dy', msg);
}

function ResibirCord(callback: (msg: any) => void) {
  subTopic("cord_dy", callback);
  
}

function bitsToDegreesXL430(bits: number): number {
  return (bits / 4095) * 360 - 180;
}

function bitsToDegreesXL320(bits: number): number {
  return (bits / 1023) * 300 - 150;
}

const Arduino = () => {
  // Estados para cada grupo de inputs (arrays de enteros)
  const [cordDy, setCordDy] = useState([2090, 2090, 2090, 512, 512, 512]);
  const [posDy, setPosDy] = useState([2090, 2090, 2090, 512, 512, 512]);
  const [pDy, setPDy] = useState([2090, 2090, 2090, 512, 512, 512]);
  
  
useEffect(() => {
  // inicializa ROS solo una vez cuando el componente se monta
  initRos();
}, [])


  //resibir msgs coordenadas de ROS
  useEffect(() => {
      ResibirCord((msg) => {  
        setPosDy(msg.data);
        setPDy(msg.data);
        setCordDy(msg.data);
      });
    }, []);
    
    
  useEffect(() => {
    if (posDy.length > 0) {
      enviarEstadoReal();
    }
  }, [cordDy]);

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
  const enviarEstadoReal = () => {
    console.log("Enviar Estado Real:", posDy);
    mandarCordReal(posDy);
    // acá tu lógica para ROS
  };

  const guardarPunto = () => {
    console.log("Guardar Punto:", pDy);
    mandarPunto(pDy);
    // acá tu lógica para ROS
  };

   return (
    
    
    
    <div style={{ padding: "20px" }}>
      <Stack direction="row" spacing={8}>
          {/* Sección 1 */}
        <div>
          <h2>Recepción cord_dy</h2>
          {cordDy.map((val, i) => (
            <div key={i} style={{ margin: "5px" }}>
              <input
                type="number"
                value={val}
                min={-1}
                max={4095}
                onChange={(e) =>
                  handleInputChange(i, e.target.value, setCordDy, cordDy)
                }
                style={{ width: "80px", marginRight: "10px" }}
              />
              {i < 3 ? (
                <span>{bitsToDegreesXL430(val).toFixed(2)}°</span>
              ) : (
                <span>{bitsToDegreesXL320(val).toFixed(2)}°</span>
              )}
            </div>
          ))}
        </div>

          {/* Sección 2 */}
        <div>
          <h2>Estado real pos_dy</h2>
          {posDy.map((val, i) => (
            <div key={i} style={{ margin: "5px" }}>
              <input
                type="number"
                value={val}
                min={-1}
                max={4095}
                onChange={(e) =>
                  handleInputChange(i, e.target.value, setPosDy, posDy)
                }
                style={{ width: "80px", marginRight: "10px" }}
              />
              {i < 3 ? (
                <span>{bitsToDegreesXL430(val).toFixed(2)}°</span>
              ) : (
                <span>{bitsToDegreesXL320(val).toFixed(2)}°</span>
              )}
            </div>
          ))}
          <br />
          <button onClick={enviarEstadoReal}>Enviar Estado Real</button>
        </div>

          {/* Sección 3 */}
        <div>
          <h2>Guardar punto p_dy</h2>
          {pDy.map((val, i) => (
            <div key={i} style={{ margin: "5px" }}>
              <input
                type="number"
                value={val}
                min={-1}
                max={4095}
                onChange={(e) => handleInputChange(i, e.target.value, setPDy, pDy)}
                style={{ width: "80px", marginRight: "10px" }}
              />
              {i < 3 ? (
                <span>{bitsToDegreesXL430(val).toFixed(2)}°</span>
              ) : (
                <span>{bitsToDegreesXL320(val).toFixed(2)}°</span>
              )}
            </div>
          ))}
          <br />
          <button onClick={guardarPunto}>Guardar Punto</button>
        </div>
        </Stack>
    </div>
  );
};

export default Arduino;
