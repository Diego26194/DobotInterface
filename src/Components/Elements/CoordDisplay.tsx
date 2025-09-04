import React, { useEffect, useState } from "react";
import { Box, Typography } from "@mui/material";
import { escucharCordReal } from "../../Services/Funciones"; 

// Definimos la estructura del mensaje esperado
interface CordMsg {
  cart: number[];   
  ang: number[];    
  error: number[];  
}

const CoordDisplay: React.FC = () => {
  const [coords, setCoords] = useState<CordMsg>({
    cart: [200, 200, 200, 200, 200, 200],
    ang: [120, 120, 210, 120, 120, 120],
    error: [0, 1, 2, 3, 4, 5], // por defecto todo en error → rojo
  });

  useEffect(() => {
    // suscribirse al tópico
    escucharCordReal((msg: CordMsg) => {
      setCoords(msg);
    });
  }, []);

  return (
    <Box
      sx={{ fontFamily: "monospace",
        border: '1px solid #000',
        borderRadius: '5px',
        padding: "1px 4px" 
      }}
    >
      {/* Primera fila → coordenadas cartesianas */}
      <Typography 
       sx={{
            fontSize: "0.65rem",   // más chico que body2
            color: "#5c5b5bff",        // gris oscuro en vez de negro
            
        }}
      >
        XYZ: {coords.cart[0]}, {coords.cart[1]}, {coords.cart[2]}{" / "}
         {coords.cart[3]}º, {coords.cart[4]}º, {coords.cart[5]}º
      </Typography>

      {/* Segunda fila → ángulos de motores, cada uno coloreado */}
      <Typography 
        sx={{
            fontSize: "0.65rem",   // más chico que body2
            color: "#5c5b5bff",        // gris oscuro en vez de negro
            
         }}
      >
        Ang:{" "}
        {coords.ang.map((val, i) => (
          <span
            key={i}
            style={{
              color: coords.error.includes(i) ? "red" : "black",
              marginRight: "2px",
            }}
          >
            {val}º
            {i < coords.ang.length - 1 ? "," : ""}
          </span>
        ))}
      </Typography>
    </Box>
  );
};

export default CoordDisplay;
