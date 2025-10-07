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
  FormLabel,
} from "@mui/material";

import { FormControl, InputLabel } from "@mui/material";

import { useState } from "react";
import InputCord from "../Elements/Inputs/InputCord";
import CompactsInputs from "../Elements/Inputs/CompactsInput";
import Button1 from "../Elements/Buettons/Button1";

import InputPositive from "../Elements/Inputs/InputPoisitive";

import CoordDisplay from "../Elements/CoordDisplay";


import InputsCartV, { InputsCartVRef } from "../Elements/Inputs/InputsCartV2";
import InputsAngV2, { InputsAngV2Ref } from "../Elements/Inputs/InputsAngV2";

import SelectPlan from "../Elements/SelectPlan";
import { SelectChangeEvent } from "@mui/material/Select";

import { agregarPuntoDB, correrTAngular, correrTCartesiano, escucharPuntoDB, agregarPuntoRutina, publicar_informe} from "../../Services/Funciones";

import CoordInput from "../Elements/Inputs/CoordInput";

//Iconos de Flechas
import ArrowForwardIosIcon from '@mui/icons-material/ArrowForwardIos';
import KeyboardDoubleArrowRightIcon from '@mui/icons-material/KeyboardDoubleArrowRight';
import TrendingFlatIcon from '@mui/icons-material/TrendingFlat';
import ArrowRightAltIcon from '@mui/icons-material/ArrowRightAlt';
import EastIcon from '@mui/icons-material/East';

import ButtonRutineAdd from "../Elements/Buettons/ButtonRutineAdd";
import ButtonSave from "../Elements/Buettons/ButtonSave";


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

  const inputsRefCart = useRef<InputsCartVRef>(null);
  const inputsRefAng = useRef<InputsAngV2Ref>(null);
  const [bloquearInputsAngV2, setBloquearInputsAngV2] = useState(false);
  const [bloquearInputsCartV, setBloquearInputsCartV] = useState(false);
  const [velocidad, setVelocidad] = useState(10);
  const [ratio, setRatio] = useState(0);
  //const [aceleracionPunto, setAceleracionPunto] = useState(0);
  const [nombrePuntoCord, setNombrePuntoCord] = useState("");
  const [plan, setPlan] = useState("PTP");
  const [diametroPuntoCord, setDiametroPuntoCord] = useState(0);

  useEffect(() => {
      escucharPuntoDB((msg) => {
        switch (msg.orden[0]) {
          case 'Punto':
            inputsRefAng.current?.setValues(msg.coordenadas.slice(0, 6));
            inputsRefCart.current?.setValues(msg.coordenadas.slice(-6));
            setNombrePuntoCord(msg.orden[1])
            blockAng();
            blockCart();
            break;
  
          case 'Ang':
            inputsRefAng.current?.setValues(msg.coordenadas);
            break;
  
          case 'Cart':
            inputsRefCart.current?.setValues(msg.coordenadas);
            break;
  
          case 'EAng':
            //inputsRefAng.current?.setValues([999,999,999,999,999,999]);
            //publicar_informe('Coodenada invalida');            
            break;
  
          case 'ECart':
            inputsRefCart.current?.setValues([999,999,999,999,999,999]);
            //publicar_informe('Coodenada invalida');            
            break;
          
          default:
            console.warn("Acción no reconocida:", msg.orden[0]);
        }});
  }, []);

  {/* 
  useEffect(() => {
    if (flagAddPoint) {
      if (inputsRefAng.current) {
        const valores = inputsRefAng.current.getValues();
        agregarPuntoDB(nombrePuntoCord, valores)
      }
      setFlagAddPoint(false);
    }
  }, [flagAddPoint, setFlagAddPoint]);
  */}

  const agregarPuDB = () => {
    if (inputsRefAng.current) {
      const valores = inputsRefAng.current.getValues();
      agregarPuntoDB(nombrePuntoCord, valores)
    }
  };

{/* REVISAR BIEN PARA ELIMINAR, la funcion de abajo ya cumple esta funcion */}
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
  

  const agregarPuRutina = () => {
    if (inputsRefAng.current) {
      const  valores = [
        ...inputsRefAng.current.getValues(),
        velocidad,
        ratio
      ];
      agregarPuntoRutina(nombrePuntoCord,plan, valores)
    }
  };

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
    setBloquearInputsCartV(true);
  };
  const activarCart = () => {
    setBloquearInputsCartV(false);
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

  const handleSetRadioCart = (val: number) => {
    setDiametroPuntoCord(val);
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
      <Stack direction="row" spacing={2}
        sx={{
          width: '100%',
          height: '4%',
          
        }}
        >
          <Typography variant="h6"sx={{fontSize: '1rem',}} >Coordenadas</Typography>
          <ButtonSave description={'Guardar Punto en Base de Datos'} onClick={agregarPuDB}/>  
          <ButtonRutineAdd description={'Agregar Punto a la Rutina'} onClick={agregarPuRutina}/> 
          

        
      </Stack>
      
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
          height: '12%',
        }}
      >
        <CoordDisplay/>
      </Box>  
      
      {/*Coordenadas a Cartesianas */}
      <FormControl 
        component="fieldset" 
        sx={{ border: '1px solid #000', 
            borderRadius: "5px", 
            padding: "0px 0px",
            fontFamily: "monospace" , 
            width: '100%' , 
            height: '27%',
        }}
      >
        <FormLabel 
          component="legend" 
          sx={{ fontSize: "0.7rem", 
            color: "#000",
            m: 0 
          }}
        >
          Coordenadas Cartesianas
        </FormLabel>

        <Stack spacing={0.5} sx={{ padding: "0px 0px"  }}>

          <Grid
            container
            spacing={2}
            sx={{
              width: '100%',
              py: 0,
              px: 1,
            }}
          >
            <Grid size={{xs: 6, md: 6}}>
              <Button1 variant="contained" onClick={activarCart}>Modificar</Button1>
            </Grid>
            <Grid size={{xs: 6, md: 6}}>
              <Button1 variant="contained" onClick={correrCordCart}>Ejecutar</Button1>
            </Grid>
          </Grid>

          <InputsCartV ref={inputsRefCart} disabled={bloquearInputsCartV}/>

        </Stack>

      </FormControl>

      
      
      {/*Coordenadas a Angulares */}
      <FormControl 
        component="fieldset" 
        sx={{ border: '1px solid #000', 
            borderRadius: "5px", 
            padding: "0px 0px",
            fontFamily: "monospace" , 
            width: '100%' , 
            height: '27%',
        }}
      >
        <FormLabel 
          component="legend" 
          sx={{ fontSize: "0.7rem", 
            color: "#000",
            m: 0 
          }}
        >
          Coordenadas Angulares
        </FormLabel>

        <Stack spacing={0.5} sx={{ padding: "0px 0px"  }}>

          <Grid
            container
            spacing={2}
            sx={{
              width: '100%',
              py: 0,
              px: 1,
            }}
          >
            <Grid size={{xs: 6, md: 6}}>
              <Button1 variant="contained" onClick={activarAng}>Modificar</Button1>
            </Grid>
            <Grid size={{xs: 6, md: 6}}>
              <Button1 variant="contained" onClick={correrCordAng}>Ejecutar</Button1>
            </Grid>
          </Grid>

          <InputsAngV2 ref={inputsRefAng} disabled={bloquearInputsAngV2}/>

        </Stack>

      </FormControl>

      {/*Caracteristicas de coordenadas */}

      <FormControl 
        component="fieldset" 
        sx={{ border: '1px solid #000', 
            borderRadius: "5px", 
            padding: "0px 0px",
            fontFamily: "monospace" , 
            width: '100%' , 
            height: '24%',
        }}
      >
        <FormLabel 
          component="legend" 
          sx={{ fontSize: "0.7rem", 
            color: "#000000ff",
            m: 0 
          }}
        >
          Caracteristicas
        </FormLabel>
        <Stack spacing={1.5} sx={{ padding: "0px 0px"  }}>

          <Grid 
            container
            spacing={2}
            sx={{
              width: '100%',
              py: 0,
              px: 1,
            }}
          >
            <Grid size={{xs: plan === 'CIRC' ? 6 : 12, md: plan === 'CIRC' ? 6 : 12}}
              sx={{
                width: '100%',
              }}>
                <SelectPlan
                  value={plan}
                  onChange={handleChangePlan}
                />
            </Grid>
            {plan === 'CIRC' ? (
              <Grid size={{xs: 6, md: 6}}
                sx={{
                  width: '100%',
                  pt: '4px'
                }}>
                  <InputPositive
                    label="Diametro"
                    valMin={0}
                    valMax={100}
                    value={diametroPuntoCord}
                    onChange={handleSetRadioCart}
                    sx={{
                      height: '100%',
                    }}
                  />                 
              </Grid>
            ):null}
          </Grid>

          <Grid
            container
            spacing={2}
            sx={{
              width: '100%',
              py: 0,
              px: 1,
            }}
          >
            <Grid size={{xs: 6, md: 6}}>
              <InputPositive
                label="Velocidad"
                valMin={0}
                valMax={100}
                value={velocidad}
                onChange={handleVelocidadChange}
              /> 
            </Grid>
            <Grid size={{xs: 6, md: 6}}>
              <InputPositive
                label="Ratio"
                valMin={0}
                valMax={100}
                value={ratio}
                onChange={handleRatioChange}
              /> 
            </Grid>
          </Grid>               
                   
          {/*<CompactsInputs label="Aceleracion" fullWidth />*/}
          <CompactsInputs 
            label="Nombre" 
            fullWidth size="small"
            value={nombrePuntoCord}
            onChange={(e) => setNombrePuntoCord(e.target.value)}
            sx={{ px: 1}}
          />
        </Stack>
      </FormControl>
      


      
    </Stack>
  );
};

export default Section2V2;
