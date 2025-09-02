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

import CompactsInputs from "../Elements/Inputs/CompactsInput";
import Button1 from "../Elements/Buettons/Button1";
import ButtonDelete from "../Elements/Buettons/ButtonDelete";
import ButtonEdit from "../Elements/Buettons/ButtonEdit";
import ButtonAdd from "../Elements/Buettons/ButtonAdd";
import SelectRutina from "../Elements/SelectRutina";
import InputCord from "../Elements/InputCord";
import InputPositive from "../Elements/Inputs/InputPoisitive";


import Section4 from "../Section1/Section4";



interface Section3Props {
  setFlagAddRutine: React.Dispatch<React.SetStateAction<boolean>>;
}
const Section3: React.FC<Section3Props> = ({ setFlagAddRutine }) => {
  const [open, setOpen] = useState(false);
  const [flagEliminarPRut, setFlagEliminarPRut] = useState(false);
  const [wait, setWait] = useState(0);
  const editwait = useRef<RutineTableRef>(null);
  const [showLibrarie, setShowLibrarie] = useState(false);

  const [flagAddPoint, setFlagAddPoint] = useState(false);

  const handleAddRutineClick = () => {
    setFlagAddRutine(true);
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
    if (editwait.current) {
      editwait.current.updateWaitForSelectedRows(wait); // llamamos la función del hijo
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
              height: '10%'
            }}
          >
          <Grid size={{xs: 10, md: 10}}>
            <Stack direction="row" spacing={1}
              sx={{
                width: '100%', //76%
              }}>

              <Typography variant="h6">Rutina</Typography>          
          
              <Button1 sx={{width: '20%'}} variant="outlined" onClick={() => setOpen(true)}>
                Correr Ruina
              </Button1>    
              <ButtonAdd description={'Agregar Punto a la Rutina'} onClick={handleAddRutineClick}/>
              <ButtonDelete description={'Eliminar Puntos de la Rutina'}onClick={handleDeleteRutineClick}/>                          
                
              <InputPositive
                label="Wait"
                valMin={0}
                valMax={100}
                value={wait}
                onChange={handleWaitChange}
                sx={{
                  fontSize: "0.75rem",
                  padding: "0px 1px",
                  minHeight: "10px",
                  maxHeight: "26px",
                  textAlign: "center",
                }}
              />     
              <Button1 sx={{width: '20%'}} variant="outlined" onClick={handleWaitPassDatagrid}>
                add
              </Button1>

              
            </Stack>

            
          </Grid>

          <Grid size={{xs: 2, md: 2}}>
            <Stack direction="row" spacing={0}>
            <CompactsInputs 
              label="Nombre de Rutina" 
              sx={{width: '80%',
                height: '100%',
                padding: '4px 0px',   
                '& input': {
                  padding: '0px 2px',
                },
              }}
            >              
            </CompactsInputs>
            <SelectRutina onAdd={handleAddClick} onEdit={handleEditClick} onDelete={handleDeleteClick }/>

            <IconButton
                onClick={() => setShowLibrarie(prev => !prev)}
                size="small"
                sx={{ "&:focus": { outline: "none" }, padding: '0px 0px', }}
              >
                {showLibrarie ?<ExpandLessIcon/> : <ExpandMoreIcon />}
                
              </IconButton>
              </Stack>
            
          </Grid>
        </Grid>  

        <Grid
          container
          spacing={1}
          sx={{
            width: '100%',
            padding: 0,
            height: '90%'
          }}
        >
          <Grid size={{xs: showLibrarie ? 10 : 12, md: showLibrarie ? 10 : 12}} >
            <RutineTable 
              flagEliminarPRut={flagEliminarPRut} 
              setFlagEliminarPRut={setFlagEliminarPRut} 
              ref={editwait} 
            />
          </Grid>

          {showLibrarie && (
          <Grid size={{xs: 2., md: 2.}} sx={{ height: '100%' , borderRight: '3px solid #aaa'}} >
            <Stack spacing={2} sx={{ height: '100%', overflowY: 'auto',}}>
              <Box sx={{ height: '100%', maxHeight: "100%"}}> 
                <Section4 setFlagAddPoint={setFlagAddPoint} />
              </Box>
            </Stack>
          </Grid>
          )}
        </Grid>
        
        
    </Stack>
  );
};

export default Section3;