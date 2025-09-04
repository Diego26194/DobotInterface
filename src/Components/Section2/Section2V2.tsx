// Section2V2.tsx
import Reac , { useRef, useEffect } from "react";
import {
  Grid,
  Typography,
  Button,
  TextField,
  Stack,
  Box,
  MenuItem,
} from "@mui/material";

import { FormControl, InputLabel } from "@mui/material";

import { useState } from "react";
import InputCord from "../Elements/Inputs/InputCord";
import CompactsInputs from "../Elements/Inputs/CompactsInput";
import Button1 from "../Elements/Buettons/Button1";

import CoordDisplay from "../Elements/CoordDisplay";


import InputsCartV2s, { InputsCartV2sRef } from "../Elements/Inputs/InputsCartV2";
import InputsAngV2, { InputsAngV2Ref } from "../Elements/Inputs/InputsAngV2";

import SelectPlan from "../Elements/SelectPlan";
import { SelectChangeEvent } from "@mui/material/Select";

import { agregarPuntoDB, correrTAngular, correrTCartesiano, escucharPuntoDB, agregarPuntoRutina } from "../../Services/Funciones";

import CoordInput from "../Elements/Inputs/CoordInput";

interface Section2V2Props {
  flagAddPoint: boolean;
  setFlagAddPoint: React.Dispatch<React.SetStateAction<boolean>>;
  flagAddRutine: boolean;
  setFlagAddRutine: React.Dispatch<React.SetStateAction<boolean>>;
}

const Section2V2: React.FC<Section2V2Props> = 
  ({ 
    flagAddPoint,
    setFlagAddPoint, 
    flagAddRutine,
    setFlagAddRutine 
  }) => {

  const inputsRefCart = useRef<InputsCartV2sRef>(null);
  const inputsRefAng = useRef<InputsAngV2Ref>(null);
  const [bloquearInputsAngV2, setBloquearInputsAngV2] = useState(false);
  const [bloquearInputsCartV2s, setBloquearInputsCartV2s] = useState(false);
  const [velocidad, setVelocidad] = useState(10);
  const [ratio, setRatio] = useState(0);
  //const [aceleracionPunto, setAceleracionPunto] = useState(0);
  const [nombrePuntoCord, setNombrePuntoCord] = useState("");
  const [plan, setPlan] = useState("PTP");

  useEffect(() => {
      escucharPuntoDB((msg) => {
        if (msg.coordenadas.length===6) {
          inputsRefAng.current?.setValues(msg.coordenadas);
          setNombrePuntoCord(msg.orden[0])
          blockAng();
      }});
  }, []);

  useEffect(() => {
      escucharPuntoDB((msg) => {
        if (msg.coordenadas.length===5) {
          inputsRefCart.current?.setValues(msg.coordenadas);
          setNombrePuntoCord(msg.orden[0])
          blockCart();
      }});
  }, []);

  useEffect(() => {
      if (flagAddPoint) {
        if (inputsRefAng.current) {
          const valores = inputsRefAng.current.getValues();
          agregarPuntoDB(nombrePuntoCord, valores)
        }
        setFlagAddPoint(false);
      }
    }, [flagAddPoint, setFlagAddPoint]);


   useEffect(() => {
      if (flagAddRutine) {
        if (inputsRefAng.current) {
          const valores = inputsRefAng.current.getValues();
          valores.push(velocidad)
          valores.push(ratio)
          agregarPuntoRutina(nombrePuntoCord,plan, valores)
        }
        setFlagAddRutine(false);
      }
    }, [flagAddRutine, setFlagAddRutine]);

  const correrCordAng = () => {
    if (inputsRefAng.current) {
      const cordAngs = inputsRefAng.current.getValues();
      correrTAngular(cordAngs);
      //console.log("Tipo de plan", plan);
      //blockAng();
    }
  };

  const correrCordCart = () => {
    //console.log("Verificar:", inputsRefCart.current);
    if (inputsRefCart.current) {
      const CordCart = inputsRefCart.current.getValues();
      correrTCartesiano(CordCart);
      //blockCart();
    }
  };

  const blockAng = () => {
    setBloquearInputsAngV2(true);
  };

  const activarAng = () => {
    setBloquearInputsAngV2(false);
    //inputsRefAng.current?.setValues([90, 45, 30, 50, 120, 60]);
    //setNombrePuntoCord('PruebaAng2')
    //setPlan('LIN');
  };

  const blockCart = () => {
    setBloquearInputsCartV2s(true);
  };
  const activarCart = () => {
    setBloquearInputsCartV2s(false);
    //inputsRefCart.current?.setValues([1,2,3,4,5]);
    //setNombrePuntoCord('pruebarcart1')
  };

  const handleChangePlan = (e: SelectChangeEvent<unknown>) => {
    setPlan(e.target.value as string);
  };

  // Handler que recibe el valor actualizado del hijo
  const handleVelocidadChange = (val: number) => {
    setVelocidad(val);
  };

  // Handler que recibe el valor actualizado del hijo
  const handleRatioChange = (val: number) => {
    setRatio(val);
  };

  //BORRARRRRRRR
  const BORRAR = (val: number | null) => {
    setRatio(val ?? 0); // si llega null, lo reemplazo por 0 (o lo que quieras)
  };

  //const [angle, setAngle] = useState(0);
  return (
    
    <Stack spacing={1}
      sx={{
        width: '100%',
        height: '100%',
        maxHeight: "100%"           
      }}
    >
      <Typography variant="h6"sx={{fontSize: '1rem',height: '5%',}} >Coordenadas</Typography>
      {/*
      <Stack direction="row" spacing={1}>
        <Button1 variant="contained">Botón 1</Button1>
        <Button1 variant="contained">Botón 2</Button1>
      </Stack>
      */}   
      {/*Coordeadas actuales */}
      <Box 
        sx={{
          width: '100%',
          height: '15%',
        }}
      >
        <CoordDisplay/>
      </Box>  
      
      {/*Coordenadas a Cartesianas */}
      <Box 
        sx={{
          width: '100%',
          height: '30%',
          border: '1px solid #000',
          borderRadius: '5px',
        }}
      >

        <Stack spacing={1}>

          <Grid
            container
            spacing={2}
            sx={{
              width: '100%',
              padding: 1,
            }}
          >
            <Grid size={{xs: 6, md: 6}}>
              <Button1 variant="contained" onClick={activarAng}>Modificar</Button1>
            </Grid>
            <Grid size={{xs: 6, md: 6}}>
              <Button1 variant="contained" onClick={correrCordAng}>Ejecutar</Button1>
            </Grid>
          </Grid>

        </Stack>

        <InputsAngV2 ref={inputsRefAng} disabled={bloquearInputsAngV2}/>

      </Box>
      
      {/*Coordenadas a Angulares */}
      <Box 
        sx={{
          width: '100%',
          height: '30%',
          border: '1px solid #000',
          borderRadius: '5px',
        }}
      >

        <Stack spacing={1}>

          <Grid
            container
            spacing={2}
            sx={{
              width: '100%',
              padding: 1,
            }}
          >
            <Grid size={{xs: 6, md: 6}}>
              <Button1 variant="contained" onClick={activarAng}>Modificar</Button1>
            </Grid>
            <Grid size={{xs: 6, md: 6}}>
              <Button1 variant="contained" onClick={correrCordAng}>Ejecutar</Button1>
            </Grid>
          </Grid>

        </Stack>

        <InputsAngV2 ref={inputsRefAng} disabled={bloquearInputsAngV2}/>

      </Box>

      {/*Caracteristicas de coordenadas */}
      <Box 
        sx={{
          width: '100%',
          height: '25%',
          border: '1px solid #000',
          borderRadius: '5px',
        }}
      >
        <Stack spacing={1}>

          <Grid 
            container spacing={0} 
            sx={{
              width: '100%', 
            }}
          >
            <Grid size={{xs: 4, md: 4}}
              sx={{
                width: '100%',
              }}>
                <Typography variant="h6">Plan: </Typography>
            </Grid>
            <Grid size={{xs: 8, md: 8}}
              sx={{
                width: '100%',
              }}>
                <SelectPlan
                  value={plan}
                  onChange={handleChangePlan}
                />
            </Grid>
          </Grid>

          <InputCord
            label="Velocidad"
            valMin={0}
            valMax={100}
            value={velocidad}
            onChange={handleVelocidadChange}
          />
          <InputCord
            label="Ratio"
            valMin={0}
            valMax={100}
            value={ratio}
            onChange={handleRatioChange}
          />
          
          {/*<CompactsInputs label="Aceleracion" fullWidth />*/}
          <CompactsInputs 
            label="Nombre" 
            fullWidth size="small"
            value={nombrePuntoCord}
            onChange={(e) => setNombrePuntoCord(e.target.value)}
          />
        </Stack>

      </Box>


      
    </Stack>
  );
};

export default Section2V2;