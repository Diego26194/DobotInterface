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
import Section2V2 from "../Components/Section2/Section2V2";
import Section3 from "../Components/Section3/Section3";

import ToolBar from "../Components/Elements/ToolBar";
import InputAng from "../Components/Elements/Inputs/InputCord";
import { useState,useEffect } from "react";
import TextField from "@mui/material/TextField";
import {initRos} from "../Services/RosService2";

const Lectura = () => {
  const [angle, setAngle] = useState(0);
  const [flagAddPoint, setFlagAddPoint] = useState(false);
  const [flagAddRutine, setFlagAddRutine] = useState(false);
  const [showSection1, setShowSection1] = useState(true);
  const [showLibrariePuntos, setShowLibrariePuntos] = useState(false);
  const [showLibrarieRutina, setShowLibrarieRutinas] = useState(false);
{/* 
  const handleRutinas = () => {
    setShowLibrarie(2)
  };

  const handlePuntos = () => {
    setShowLibrarie(1)
  };

  const handleSection1 = () => {
    setShowLibrarie(0)
  };
*/}
useEffect(() => {
  // inicializa ROS solo una vez cuando el componente se monta
  initRos();
}, [])
  return (
    <Stack
      sx={{
        display: 'flex',
        height: '100vh',
        width: '100vw',  
      }}
    > 
      {/*//Barra de herramientas*/} 
      <Stack direction="row" spacing={5}  
        sx={{ height: '3%' , width: '100%', backgroundColor: 'rgba(194, 170, 104, 0.76)',}}
      >
        <Button 
          onClick={() => setShowLibrariePuntos(prev => !prev)} 
          variant="contained" 
          sx={{ height: '85%'}}
        >
          B.Puntos
        </Button>
        <Button 
          onClick={() => setShowLibrarieRutinas(prev => !prev)} 
          variant="contained" 
          sx={{ height: '85%'}}
        >
          B.Rutinas
        </Button>
        <Button 
         // onClick={handleRutinas} 
          variant="contained" 
          sx={{ height: '85%'}}
        >
          Inicializar Programa
        </Button>
        <Button 
          //onClick={handleRutinas} 
          variant="contained" 
          sx={{ height: '85%'}}
        >
          Cerrar Programa
        </Button>

        <Button 
         // onClick={handleRutinas} 
          variant="contained" 
          sx={{ height: '85%'}}
        >
          M Escritura
        </Button>
        <Button 
         // onClick={handleRutinas} 
          variant="contained" 
          sx={{ height: '85%'}}
        >
          M Lectura
        </Button>

      </Stack>
    
      <Grid container spacing={0} sx={{ height: '86%' , width: '100%' }}>
        
        {/*Section1 */}
        {(showLibrariePuntos || showLibrarieRutina) && (
          <Grid size={{xs: 1.8, md: 1.8}} sx={{ height: '100%' , borderRight: '3px solid #aaa'}} >
            <Stack spacing={0.5} sx={{ height: '100%', overflowY: 'auto',}}>
              {showLibrariePuntos && (
                <Box sx={{ height: showLibrarieRutina? '49%':'100%', maxHeight: showLibrarieRutina? '49%':'100%'}}> 
                    <Section1 setFlagAddPoint={setFlagAddPoint} />
                </Box>
              )}
              {showLibrarieRutina && (
                <Box sx={{ height: showLibrariePuntos? '49%':'100%', maxHeight: showLibrariePuntos? '49%':'100%'}}> 
                  <Section4 setFlagAddPoint={setFlagAddPoint} />
                </Box>
              )}
              
            </Stack> 
          </Grid>
        )}

        <Grid 
          size={{ xs: (showLibrariePuntos || showLibrarieRutina) ? 10.2 : 12, 
            md: (showLibrariePuntos || showLibrarieRutina) ? 10.2 : 12
          }}
          sx={{ 
            height: '100%',
            display: 'flex',
            flexDirection: 'column', 
          }} 
        >
          <Grid container spacing={0} sx={{ height: '100%' , width: '100%' }}>
            <Grid 
              size={{ xs: 2.3,md: 2.2}}
              sx={{ 
                height: '100%' ,
                backgroundColor: '#d0eaff',
                borderBottom: '4px solid #aaa',
                //overflow: 'auto',
                //borderLeft: '10px solid #aaa',
                padding:"5px",
                overflowY: 'auto'
              }} 
            >
              <Section2V2
                flagAddPoint={flagAddPoint} 
                setFlagAddPoint={setFlagAddPoint}
                flagAddRutine={flagAddRutine} 
                setFlagAddRutine={setFlagAddRutine}
              />
            </Grid>

            <Grid 
              size={{ xs: 9.7,md: 9.8}}
              sx={{ 
                height: '100%' ,
                //backgroundColor: '#FF745C',
                //borderBottom: '4px solid #aaa',
                //overflow: 'auto',
                padding:"4px",
              }} 
            >
              <Section3 setFlagAddRutine={setFlagAddRutine} />
            </Grid>

          </Grid>
          
          
        </Grid>
      </Grid>
      <Box
        sx={{
          width: '100%', 
          height: '10%',
          display: 'flex',
          border: '1px solid #000',
          backgroundColor: '#dbdbdbe5',
        }}
      ></Box>
    </Stack>
  );
};

export default Lectura;

