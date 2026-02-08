import React, { useState, useRef } from "react";
import {
  Typography,
  Box,
  IconButton,
  TableBody,
  TableCell,
  TableHead,
  TableRow,
  Paper,
  Grid,
  Button,
  Stack,
  Dialog,
  DialogTitle,
  DialogContent,
  DialogActions
} from "@mui/material";
import RutineTable, { RutineTableRef } from '../../Pages/RutineTable';

import ExpandMoreIcon from "@mui/icons-material/ExpandMore";
import ExpandLessIcon from "@mui/icons-material/ExpandLess";
import PlayArrowIcon from '@mui/icons-material/PlayArrow';

import CompactsInputs from "../Elements/Inputs/CompactsInput";
import Button1 from "../Elements/Buettons/Button1";
import ButtonDelete from "../Elements/Buettons/ButtonDelete";
import ButtonEdit from "../Elements/Buettons/ButtonEdit";
import ButtonAdd from "../Elements/Buettons/ButtonAdd";
import ButtonSave from "../Elements/Buettons/ButtonSave";
import ButtonReflesh from "../Elements/Buettons/ButtonReflesh";
import SelectRutina from "../Elements/SelectRutina";
import InputCord from "../Elements/Inputs/InputCord";
import InputPositive from "../Elements/Inputs/InputPoisitive";

import ArrowRightAltIcon from '@mui/icons-material/ArrowRightAlt';

import Section4 from "../Section1/Section4";
import { ejecutarRutina, refrescarRutina, agregarRutina} from "../../Services/Funciones";


interface Section3Props {
  setFlagAddRutine: React.Dispatch<React.SetStateAction<boolean>>;
  modoActuar: boolean;
}
const Section3: React.FC<Section3Props> = ({ setFlagAddRutine, modoActuar }) => {
  const [open, setOpen] = useState(false);
  const [flagEliminarPRut, setFlagEliminarPRut] = useState(false);
  const [wait, setWait] = useState(0);
  const tableRef = useRef<RutineTableRef>(null);
  const [showLibrarie, setShowLibrarie] = useState(false);
  const [nombreRutina, setNombreRutina] = useState("");
  

  const [flagAddPoint, setFlagAddPoint] = useState(false);

  const handleAddRutineClick = () => {
    //setFlagAddRutine(true);
    agregarRutina(nombreRutina)
  };

  const handleDeleteRutineClick = () => {
    setFlagEliminarPRut(true);
  };

  const handleAddClick = () => {
    alert(`Click add`);
  };

  const handleEditClick = () => {
    alert(`Click Edit`);
  };

  const handleDeleteClick = () => {
    alert(`Click Delete`);
  };

  // Handler que recibe el valor actualizado del hijo
  const handleWaitChange = (val: number) => {
    setWait(val);
  };
  
  // Handler que recibe el valor actualizado del hijo
  const handleWaitPassDatagrid = () => {
    if (tableRef.current) {
      tableRef.current.updateWaitForSelectedRows(wait); 
    }
  };

  const handleEjecutarClick = () => {
    if (tableRef.current?.validarRutina()) {
      ejecutarRutina();
    } else {
      alert("⚠ Hay filas en modo edición. Confirme o cancele antes de ejecutar.");
    }
  };


  

  <Dialog open={open} onClose={() => setOpen(false)}>
    <DialogTitle>Subtítulo del diálogo</DialogTitle>
    <DialogContent>
      <Typography>Contenido del diálogo.</Typography>
    </DialogContent>
    <DialogActions>
      <Button1 onClick={() => setOpen(false)}>Cerrar</Button1>
    </DialogActions>
  </Dialog>

  return (
    <Stack  spacing={0}
      sx={{
          width: '100%', 
          height: '100%'
        }}>
          <Grid
            container
            spacing={1}
            sx={{
              width: '100%',
              padding: 0,
              height: '7%'
            }}
          >
          <Grid size={{xs: 10, md: 10}}>
            <Stack direction="row" spacing={1}
              sx={{
                width: '100%', //76%
              }}>

              <Typography variant="h6">Rutina</Typography>  

              <CompactsInputs 
                label="Nombre de Rutina" 
                
                fullWidth size="small"
                value={nombreRutina}
                onChange={(e) => setNombreRutina(e.target.value)}
                sx={{width: '40%',
                  height: '100%',
                  padding: '4px 0px',   
                  '& input': {
                    padding: '0px 2px',
                  },
                }}
              >              
              </CompactsInputs>        

              <ButtonSave description={'Guardar Rutina'} onClick={handleAddRutineClick}/>
          
              <Button1 title= "Ejecutar Rutina" sx={{width: '20%'}} variant="outlined" onClick={handleEjecutarClick} disabled={!modoActuar}>
                {<PlayArrowIcon />}Ejecutar
              </Button1>    
             {/*<ButtonAdd description={'Agregar Punto a la Rutina'} onClick={handleAddRutineClick}/> */}
              <ButtonDelete description={'Eliminar Instrucciones seleccionada de la Rutina'}onClick={handleDeleteRutineClick}/>                          
                
              <InputPositive
                label="Wait" 
                valMin={0}
                valMax={100}
                value={wait}
                onChange={handleWaitChange}
                sx={{
                  fontSize: "0.9rem",
                  padding: "0px 1px",
                  minHeight: "10px",
                  maxHeight: "30px",
                  textAlign: "center",
                  pt: 1,
                }}
              />     
              <Button1 title={'Agregar Tiempo de espera en intruccion seleccionada'} sx={{width: '20%'}} variant="outlined" onClick={handleWaitPassDatagrid}>
                add
              </Button1>
              
              <ButtonReflesh description={'Refrescar tabla de rutina'}onClick={refrescarRutina }/> 
              
            </Stack>

            
          </Grid>

          <Grid size={{xs: 2, md: 2}}>
            <Stack direction="row" spacing={0}>
              
              {/* 
              <SelectRutina onAdd={handleAddClick} onEdit={handleEditClick} onDelete={handleDeleteClick }/>

              <IconButton
                  onClick={() => setShowLibrarie(prev => !prev)}
                  size="small"
                  sx={{ "&:focus": { outline: "none" }, padding: '0px 0px', }}
                >
                  {showLibrarie ?<ExpandLessIcon/> : <ExpandMoreIcon />}
                  
              </IconButton>
            */}
            </Stack>
            
          </Grid>
        </Grid>  

        <Grid
          container
          spacing={1}
          sx={{
            width: '100%',
            padding: 0,
            height: '93%'
          }}
        >
          <Grid size={{xs: showLibrarie ? 10 : 12, md: showLibrarie ? 10 : 12}} >
            <RutineTable 
              flagEliminarPRut={flagEliminarPRut} 
              setFlagEliminarPRut={setFlagEliminarPRut} 
              ref={tableRef} 
              modoActuar={modoActuar}
            />
          </Grid>
          {/* 
          {showLibrarie && (
          <Grid size={{xs: 2., md: 2.}} sx={{ height: '100%' , borderRight: '3px solid #aaa'}} >
            <Stack spacing={2} sx={{ height: '100%', overflowY: 'auto',}}>
              <Box sx={{ height: '100%', maxHeight: "100%"}}> 
                <Section4 setFlagAddPoint={setFlagAddPoint} />
              </Box>
            </Stack>
          </Grid>
          )}
          */}
        </Grid>
        
        
    </Stack>
  );
};

export default Section3;