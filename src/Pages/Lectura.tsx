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

import ToolBar from "../Components/Elements/ToolBar2";
import InputAng from "../Components/Elements/Inputs/InputCord";
import { useState,useEffect, useRef } from "react";
import TextField from "@mui/material/TextField";
import {initRos} from "../Services/RosService2";
import {msgInforme, ModoActuar} from "../Services/Funciones";

type MensajeInforme = {
  texto: string;
  nivel: number;
};

const Lectura = () => {
  const [angle, setAngle] = useState(0);
  const [flagAddPoint, setFlagAddPoint] = useState(false);
  const [flagAddRutine, setFlagAddRutine] = useState(false);
  const [showSection1, setShowSection1] = useState(true);
  const [showLibrariePuntos, setShowLibrariePuntos] = useState(false);
  const [showLibrarieRutina, setShowLibrarieRutinas] = useState(false);

  const boxRef = useRef<HTMLDivElement>(null);

  const [mensajes, setMensajes] = useState<MensajeInforme[]>([]);

  const [modoActuar, setModoActuar] = useState<boolean>(true);
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

const getColorByNivel = (nivel: number): string => {
  switch (nivel) {
    case 0:   // informativo
      return "#f1f1eb"; // blanco suave
    case 1:   // OK
      return "#4caf50"; // verde
    case -1:  // advertencia
      return "#ffb300"; // amarillo ámbar
    case -2:  // error
      return "#e53935"; // rojo
    default:
      return "#b0bec5"; // gris por defecto
  }
};

useEffect(() => {
  ModoActuar((msg: any) => {
    if (typeof msg.data === "boolean") {
      setModoActuar(msg.data);
    } else {
      console.warn("modo_actuar inválido:", msg);
    }
  });
}, []);

useEffect(() => {
  setMensajes([
    {
      texto: "Prueba de inicialización",
      nivel: -2,
    },
  ]);
}, []);

//scroll valla siempre al final al llegar un msg
useEffect(() => {
    if (boxRef.current) {
      // Mueve el scroll al fondo
      boxRef.current.scrollTop = boxRef.current.scrollHeight;
    }
  }, [mensajes]);


useEffect(() => {
  // inicializa ROS solo una vez cuando el componente se monta
  initRos();
}, [])

useEffect(() => {
  msgInforme((msg: any) => {
    if (
      msg &&
      typeof msg.texto === "string" &&
      typeof msg.nivel === "number"
    ) {
      setMensajes((prev) => {
        const updated = [...prev, { texto: msg.texto, nivel: msg.nivel }];
        return updated.length > 50 ? updated.slice(-50) : updated;
      });
    } else {
      console.warn("Mensaje recibido con formato inválido:", msg);
    }
  });
}, []);

  return (
    <Stack
      sx={{
        display: 'flex',
        height: '100vh',
        width: '100vw',  
      }}
    > 

      <ToolBar 
        LinkHome="/arduino" 
        onTogglePuntos={() => setShowLibrariePuntos(p => !p)}
        onToggleRutinas={() => setShowLibrarieRutinas(p => !p)}
        modoActuar={modoActuar}
        />
      {/*//Barra de herramientas*/} 
      {/*
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
      */} 
    
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
                modoActuar={modoActuar}
              />
            </Grid>

            <Grid 
              size={{ xs: 9.7,md: 9.8}}
              sx={{ 
                height: '100%' ,
                //backgroundColor: '#81B8D1',
                //borderBottom: '4px solid #aaa',
                //overflow: 'auto',
                padding:"4px",
              }} 
            >
              <Section3 setFlagAddRutine={setFlagAddRutine} modoActuar={modoActuar}/>
              
            </Grid>

          </Grid>
          
          
        </Grid>
      </Grid>
      <Box
        ref={boxRef}
        sx={{
          width: "100%",
          height: "10%",
          display: "flex",
          flexDirection: "column",  // para que los mensajes se apilen
          overflowY: "auto",        // scroll si hay muchos
          border: "1px solid #0adf1cff",
          backgroundColor: "#1a1a1a",
          p: 1,
        }}
      >
        {mensajes.map((m, idx) => (
          <Typography
            key={idx}
            variant="body2"
            sx={{
              color: getColorByNivel(m.nivel),
              fontSize: "0.75rem",
              lineHeight: 1.4,
              fontFamily: "monospace",
            }}
          >
           - {m.texto}
          </Typography>
        ))}
      </Box>
    </Stack>
  );
};

export default Lectura;

