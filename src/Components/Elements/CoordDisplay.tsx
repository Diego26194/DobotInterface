import React, { useEffect, useState } from "react";
import { Box, Typography, FormLabel, FormControl } from "@mui/material";
import { escucharCordReal } from "../../Services/Funciones"; 

// Definimos la estructura del mensaje esperado
interface CordMsg {
  cart: number[];   
  ang: number[];    
  error: boolean[];  
}

const CoordDisplay: React.FC = () => {
  const [coords, setCoords] = useState<CordMsg>({
    cart: [200, 200, 200, 200, 200, 200],
    ang: [120, 120, 210, 120, 120, 120],
    error: [false, false, false, false, false, false], // error: [false, false, false, false, false, false] error: [true, true, true, true, true, true]
  });

  useEffect(() => {
    // suscribirse al tópico
    escucharCordReal((msg: CordMsg) => {
      console.log(msg.cart);
      console.log(msg.ang);
      console.log(msg.error);

      setCoords(msg);
    });
  }, []);

  return (
    <FormControl 
      component="fieldset" 
      sx={{ border: '1px solid #000', 
          borderRadius: "5px", 
          padding: "0px 0px",
          fontFamily: "monospace" , 
          width: '100%' , 
          height: '100%',
      }}
    >
      <FormLabel 
        component="legend" 
        sx={{ fontSize: "0.7rem", 
          color: coords.error.includes(false) ? "red" : "green", 
        }}
      >
        Coordenadas Reales
      </FormLabel>

      <Box sx={{ padding: "0px 3px"  }} >
        {/* Primera fila → coordenadas cartesianas */}
        <Typography 
        sx={{
              fontSize: "0.7rem",   // más chico que body2
              color: "#0a0a0aff",        // gris oscuro en vez de negro
              
          }}
        >
          XYZ: {coords.cart[0]}, {coords.cart[1]}, {coords.cart[2]}{" / "}
          {coords.cart[3]}º, {coords.cart[4]}º, {coords.cart[5]}º
        </Typography>

        {/* Segunda fila → ángulos de motores, cada uno coloreado */}
        <Typography 
          sx={{
              fontSize: "0.7rem",   // más chico que body2
              color: "#5c5b5bff",        // gris oscuro en vez de negro
              
          }}
        >
          Ang:{" "}
          {coords.ang.map((val, i) => (
            <span
              key={i}
              style={{
                color: coords.error[i] ? "#5c5b5bff" : "red"  ,
                marginRight: "2px",
              }}
            >
              {val}º
              {i < coords.ang.length - 1 ? "," : ""}
            </span>
          ))}
        </Typography>
      </Box>
    </FormControl>
  );
};

export default CoordDisplay;
