import React, { useState, useEffect } from "react";
import {initRos, pubTopic, subTopic, unsubTopic } from '../Services/RosService2';
import {  Stack } from "@mui/material";

function cart_ang( coord:number[]) {
  const msg = {
    data: coord
  };

  //console.log(" Enviando a /pos_dy: correr angulos", coord);

  // Publica en ROS
  pubTopic('cart_ang', msg);
}

function ang_cart( coord:number[]) {
  const msg = {
    data: coord
  };

  console.log(" Enviando a /pos_dy: correr angulos", coord);

  // Publica en ROS
  pubTopic('ang_cart', msg);
}

function punto_ang(callback: (msg: any) => void) {
  subTopic("ang", callback);
  
}

function punto_cart(callback: (msg: any) => void) {
  subTopic("cart", callback);
  
}

function punto_pose(callback: (msg: any) => void) {
  subTopic("pose", callback);
  
}

function bitsToDegrees(bits: number): number {
  return (bits / 4095) * 360 - 180;
}

const Cinematica = () => {
  // Estados para cada grupo de inputs (arrays de enteros)
  const [ang, setAng] = useState([0,0,0,0,0,0]);
  const [cart, setCart] = useState([0,0,0,0,0,0]);
  const [pose, setPose] = useState([0,0,0,0,0,0,0]);
  
  
useEffect(() => {
  // inicializa ROS solo una vez cuando el componente se monta
  initRos();
}, [])


  //resibir msgs coordenadas de ROS
  useEffect(() => {
      punto_ang((msg) => {  
        setAng(msg.data);
      });
    }, []);

  //resibir msgs coordenadas de ROS
  useEffect(() => {
      punto_pose((msg) => {  
        setPose(msg.data);
      });
    }, []);

  //resibir msgs coordenadas de ROS
  useEffect(() => {
      punto_cart((msg) => {  
        setCart(msg.data);
      });
    }, []);
    
    
  useEffect(() => {
    if (cart.length > 0) {
      enviarCart();
    }
  }, [ang]);

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
  const enviarCart = () => {
    console.log("Enviar Estado Real:", cart);
    cart_ang(cart);
    // acá tu lógica para ROS
  };

  const enviarAng = () => {
    console.log("Guardar Punto:", ang);
    ang_cart(ang);
    // acá tu lógica para ROS
  };

   return (
    
    
    
    <div style={{ padding: "20px" }}>
      <Stack direction="row" spacing={8}>
          {/* Sección 1 */}
        <div>
          <h2>psoe</h2>
          {pose.map((val, i) => (
            <div key={i} style={{ margin: "5px" }}>
              <input
                type="number"
                value={val}
                min={-1}
                max={4095}
                onChange={(e) =>
                  handleInputChange(i, e.target.value, setAng, pose)
                }
                style={{ width: "80px", marginRight: "10px" }}
              />
              <span>{bitsToDegrees(val).toFixed(2)}°</span>
            </div>
          ))}
        </div>

          {/* Sección 2 */}
        <div>
          <h2>cartesiana</h2>
          {cart.map((val, i) => (
            <div key={i} style={{ margin: "5px" }}>
              <input
                type="number"
                value={val}
                min={-1}
                max={4095}
                onChange={(e) =>
                  handleInputChange(i, e.target.value, setCart, cart)
                }
                style={{ width: "80px", marginRight: "10px" }}
              />
              <span>{bitsToDegrees(val).toFixed(2)}°</span>
            </div>
          ))}
          <br />
          <button onClick={enviarCart}>enviarCart</button>
        </div>

          {/* Sección 3 */}
        <div>
          <h2>angulos</h2>
          {ang.map((val, i) => (
            <div key={i} style={{ margin: "5px" }}>
              <input
                type="number"
                value={val}
                min={-1}
                max={4095}
                onChange={(e) => handleInputChange(i, e.target.value, setPose, ang)}
                style={{ width: "80px", marginRight: "10px" }}
              />
              <span>{bitsToDegrees(val).toFixed(2)}°</span>
            </div>
          ))}
          <br />
          <button onClick={enviarAng}>enviarAng</button>
        </div>
        </Stack>
    </div>
  );
};

export default Cinematica;
