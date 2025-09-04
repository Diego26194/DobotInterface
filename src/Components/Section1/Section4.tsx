// Section1.tsx
import React, { useState, useEffect, useRef } from "react";
import {
  Button,
  Box,
  Typography,
  Stack,
  Table,
  TableBody,
  TableCell,
  TableHead,
  TableRow,
  Paper
} from "@mui/material";

import Button1 from "../Elements/Buettons/Button1";
import ButtonDelete from "../Elements/Buettons/ButtonDelete";
import ButtonEdit from "../Elements/Buettons/ButtonEdit";
import ButtonAdd from "../Elements/Buettons/ButtonAdd";
import ButtonReflesh from "../Elements/Buettons/ButtonReflesh";


import RutineLibrarie from "../Elements/RutineLibrarie";

import { cargarPuntosDB, elimiarPuntoDB, escucharPuntoDB } from "../../Services/Funciones";


interface Section4Props {
  setFlagAddPoint: React.Dispatch<React.SetStateAction<boolean>>;
}

const Section4: React.FC<Section4Props> = ({ setFlagAddPoint }) => {
  const [open, setOpen] = useState(false);
  const [flagDelete, setFlagDelete] = useState(false);
  const [flagAdd, setFlagAdd] = useState(false);
  const [nombre, setNombre] = useState('')

  const handleDeleteClick = () => {
    setFlagDelete(true); // Esto activa el efecto en PointsTable
  };

  const handleAddClick = () => {
    setFlagAdd(true); // Esto activa el efecto en PointsTable
    setFlagAddPoint(true);
  };
    
  const handleSeleccion = (name: string) => {
    console.log('Seleccionado:', name);
    setNombre(name)
  };

  return (
    
    <Stack 
      spacing={0}
      sx={{
        width: '100%', 
        height: '100%', 
      }}
    >
      {/*<Button variant="contained" onClick={cargarPuntosDB}>Cargar datos</Button>*/}
      <Stack direction="row" spacing={0} sx={{height: '10%'}}>
        <Typography variant="h6"sx={{fontSize: '1.1rem'}}>Rutina</Typography>
        <ButtonAdd onClick={handleAddClick}/>
        <ButtonEdit onClick={() => setOpen(true)}/>
        <ButtonDelete onClick={() => elimiarPuntoDB(nombre)}/>
        <ButtonReflesh onClick={cargarPuntosDB}/>
      </Stack>
      <Paper sx={{
            width: '100%', 
            height: '90%',
            border: '1px solid #000',
          }}> 

          <RutineLibrarie  onSelect={handleSeleccion} flagDelete={flagDelete} setFlagDelete={setFlagDelete} 
          flagAdd={flagAdd} setFlagAdd={setFlagAdd}
          />
          {/*
        <Table 
          size="small" 
          
        >
          
          <TableHead>
            <TableRow>
              <TableCell>Columna 1</TableCell>
              <TableCell>Columna 2</TableCell>
            </TableRow>
          </TableHead>
          <TableBody>
            <TableRow>
              <TableCell>Dato 1</TableCell>
              <TableCell>Dato 2</TableCell>
            </TableRow>
          </TableBody>
        </Table>
          */}
      </Paper>

      

    </Stack>
  );
};

export default Section4;