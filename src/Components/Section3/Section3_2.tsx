import React, { useState } from "react";
import {
  Typography,
  Table,
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
import RutineTable from '../../Pages/RutineTable';

import CompactsInputs from "../Elements/Inputs/CompactsInput";
import Button1 from "../Elements/Buettons/Button1";
import ButtonDelete from "../Elements/Buettons/ButtonDelete";
import ButtonEdit from "../Elements/Buettons/ButtonEdit";
import ButtonAdd from "../Elements/Buettons/ButtonAdd";


interface Section3Props {
  setFlagAddRutine: React.Dispatch<React.SetStateAction<boolean>>;
}
const Section3: React.FC<Section3Props> = ({ setFlagAddRutine }) => {
  const [open, setOpen] = useState(false);

  const handleAddClick = () => {
    setFlagAddRutine(true);
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
    <Stack  spacing={1}
    sx={{
          width: '100%', 
          height: '100%'
        }}>
        <Stack direction="row" spacing={1} 
        sx={{
          width: '100%', //76%
        }}>
          <Typography variant="h6">Rutina</Typography>
          <CompactsInputs label="Nombre de Rutina"  /> 
          <ButtonAdd onClick={handleAddClick}/>
          <ButtonEdit onClick={() => setOpen(true)}/>
          <ButtonDelete onClick={() => setOpen(true)}/>
        </Stack>
      <Grid 
        container spacing={2} 
        sx={{
          width: '100%', 
          height: '100%'
        }}
      >
        <Grid size={{xs: 1.5, md: 1.5}}>
          <Stack  spacing={1}>
            
            <CompactsInputs label="Velocidad" fullWidth />
            <CompactsInputs label="Aceleracion" fullWidth />
            <CompactsInputs label="Ratio" fullWidth />             
            <Stack direction="row" spacing={1}>
              <ButtonAdd onClick={handleAddClick}/>
              <ButtonDelete onClick={() => setOpen(true)}/>
            </Stack>
            <Button1 sx={{width: '110%'}} variant="outlined" onClick={() => setOpen(true)}>
              Correr Ruina
            </Button1>
          </Stack>
        </Grid>

        <Grid size={{xs: 10.5, md: 10.5/*xs: 7.5, md: 7.5*/}}>
          <RutineTable/>
        </Grid>
      </Grid>
    </Stack>
  );
};

export default Section3;