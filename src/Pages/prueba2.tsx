import React from 'react';
import { Box } from '@mui/material';

import Section1 from "../Components/Section1/Section1";
import Section2 from "../Components/Section2/Section2";
import Section3 from "../Components/Section3/Section3";

const Prueba2 = () => {
  return (
    <Box
      sx={{
        display: 'flex',
        height: '100vh',
        width: '100vw',  
        overflow: 'hidden',
      }}
    >
      {/* Sección 1: izquierda */}
      <Box
        sx={{
          width: '20.83%', // 2.5 de 12
          height: '100%',
          backgroundColor: '#f0f0f0',
          borderRight: '15px solid #ccc',
          //overflow: 'auto',
        }}
      >
        <Section1 />
      </Box>

      {/* Contenedor de S2 y S3 */}
      <Box
        sx={{
          width: '79.17%', // 9.5 de 12
          height: '100%',
          display: 'flex',
          flexDirection: 'column',
        }}
      >
        {/* S2: arriba derecha */}
        <Box
          sx={{
            height: '33.33%', // 4 de 12
            backgroundColor: '#d0eaff',
            borderBottom: '1px solid #aaa',
            borderLeft: '10px solid #aaa',
            overflow: 'auto',
          }}
        >
          <Section2 />
        </Box>

        {/* S3: abajo derecha */}
        <Box
          sx={{
            height: '66.67%', // 8 de 12
            backgroundColor: '#fff4d0',
            overflow: 'auto',
          }}
        >
          <Section3 />
        </Box>
      </Box>
    </Box>
  );
}
export default Prueba2;