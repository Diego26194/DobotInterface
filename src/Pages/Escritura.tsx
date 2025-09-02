import React from "react";
//import { Grid, Container } from "@mui/material";
import Container from '@mui/material/Container';

import {
  Typography,
  Box,
  TableBody,
  TableCell,
  TableHead,
  TableRow,
  Grid,
  Button,
  Stack,
  Dialog,
  DialogTitle,
  DialogContent,
  DialogActions
} from "@mui/material";
import Section1 from "../Components/Section1/Section1";
import Section4 from "../Components/Section1/Section4";
import Section2 from "../Components/Section2/Section2";
import Section3 from "../Components/Section3/Section3";

import ToolBar from "../Components/Elements/ToolBar";
import InputAng from "../Components/Elements/InputCord";
import { useState } from "react";
import TextField from "@mui/material/TextField";

const Escritura = () => {
  const [angle, setAngle] = useState(0);
  const [flagAddPoint, setFlagAddPoint] = useState(false);
  const [flagAddRutine, setFlagAddRutine] = useState(false);
  const [showSection1, setShowSection1] = useState(true);
  const [showLibrarie, setShowLibrarie] = useState(0);

  const handleRutinas = () => {
    setShowLibrarie(2)
  };

  const handlePuntos = () => {
    setShowLibrarie(1)
  };

  const handleSection1 = () => {
    setShowLibrarie(0)
  };

  return (
    <Stack
      sx={{
        display: 'flex',
        height: '100vh',
        width: '100vw',  
        //overflow: 'hidden',
        //padding: 1,
        //boxSizing: 'border-box',
        //backgroundColor: '#fff4d0',
      }}
    > 
    {/* Botonera de prueba */}
    {/*  
      <Stack direction="row" spacing={12}>
        <Button 
          onClick={handleSection1} 
          variant="contained" 
        >
          {showSection1 ? "Ocultar Panel" : "Mostrar Panel"}
        </Button>
        <Button 
          onClick={handlePuntos} 
          variant="contained" 
        >
          biblioteca de Putnos
        </Button>
        <Button 
          onClick={handleRutinas} 
          variant="contained" 
        >
          biblioteca de rutinas
        </Button>
        
        {/*<ToolBar LinkHome="/lectura" />


      </Stack> */}
      <Stack direction="row" spacing={5}
        sx={{ height: '4%' , width: '100%', backgroundColor: 'rgba(194, 170, 104, 0.76)',}}
      >
       <Button 
          onClick={handleSection1} 
          variant="contained" 
          sx={{ height: '85%'}}
        >
          {showSection1 ? "Ocultar Panel" : "Mostrar Panel"}
        </Button>
        <Button 
          onClick={handlePuntos} 
          variant="contained" 
          sx={{ height: '85%'}}
        >
          B.Putnos
        </Button>
        <Button 
          onClick={handleRutinas} 
          variant="contained" 
          sx={{ height: '85%'}}
        >
          B.Rutinas
        </Button>
        <Button 
          onClick={handleRutinas} 
          variant="contained" 
          sx={{ height: '85%'}}
        >
          Inicializar Programa
        </Button>
        <Button 
          onClick={handleRutinas} 
          variant="contained" 
          sx={{ height: '85%'}}
        >
          Cerrar Programa
        </Button>

        <Button 
          onClick={handleRutinas} 
          variant="contained" 
          sx={{ height: '85%'}}
        >
          M Escritura
        </Button>
        <Button 
          onClick={handleRutinas} 
          variant="contained" 
          sx={{ height: '85%'}}
        >
          M Lectura
        </Button>

      </Stack>
    
      <Grid container spacing={0} sx={{ height: '96%' , width: '100%' }}>
        {/*<Button 
          onClick={() => setShowSection1(prev => !prev)} 
          variant="contained" 
          sx={{ position: "absolute", top: 10, left: 10, zIndex: 1000 }}
        >
          {showSection1 ? "Ocultar Panel" : "Mostrar Panel"}
        </Button>*/}

        {/* Section1 condicional */}
        {showLibrarie > 0 ? (
          <Grid size={{xs: 2.5, md: 2.5}} sx={{ height: '100%' , borderRight: '3px solid #aaa'}} >
            <Stack spacing={2} sx={{ height: '100%', overflowY: 'auto',}}>
              <Box sx={{ height: '80%', maxHeight: "80%"}}> 
                {showLibrarie === 1 ? (
                  <Section1 setFlagAddPoint={setFlagAddPoint} />
                ) : showLibrarie === 2 ? (
                  <Section4 setFlagAddPoint={setFlagAddPoint} />
                ) : null}      
              </Box>
              <Box
                  sx={{
                    width: '100%', 
                    height: '16%',
                    display: 'flex',
                    border: '1px solid #000',
                  }}
                ></Box>
            </Stack> 
          </Grid>
        ): null}

        <Grid size={{ xs: showLibrarie ? 9.5 : 12, md: showLibrarie ? 9.5 : 12}} sx={{ height: '100%' }} >
          <Box
            sx={{
              width: '100%', 
              height: '100%',
              display: 'flex',
              flexDirection: 'column',
            }}
          >
            {/* S2: arriba derecha */}
            <Box
              sx={{
                height: '33%',//'33.33%', // 4 de 12
                backgroundColor: '#d0eaff',
                borderBottom: '4px solid #aaa',
                //overflow: 'auto',
                //borderLeft: '10px solid #aaa',
                padding:"5px",
                overflowY: 'auto'
              }}
            >
              <Section2 
                flagAddPoint={flagAddPoint} 
                setFlagAddPoint={setFlagAddPoint}
                flagAddRutine={flagAddRutine} 
                setFlagAddRutine={setFlagAddRutine}
              />
            </Box>

            {/* S3: abajo derecha */}
            <Box
              sx={{
                height: '67%', // 8 de 12
                //backgroundColor: '#FF745C',
                //borderBottom: '4px solid #aaa',
                //overflow: 'auto',
                padding:"4px",
              }}
            >
              <Section3 setFlagAddRutine={setFlagAddRutine} />
              
            </Box>
          </Box>
          
          </Grid>
      </Grid>
    </Stack>
  );
};

export default Escritura;

