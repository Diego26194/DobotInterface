import React, { useState, useEffect } from 'react';
import {
  Table, TableBody, TableCell, TableContainer,
  TableHead, TableRow, Paper
} from '@mui/material';
import { nombrePuntoDB } from "../../Services/Funciones";

interface Point {
  nombre: string;
}

interface RutineLibrarieProps {
  onSelect: (nombre: string) => void;  // Para pasar el nombre al hacer clic
  flagDelete: boolean;
  setFlagDelete: React.Dispatch<React.SetStateAction<boolean>>;
  flagAdd: boolean;
  setFlagAdd: React.Dispatch<React.SetStateAction<boolean>>;
}

const RutineLibrarie: React.FC<RutineLibrarieProps> = ({ onSelect, flagDelete, setFlagDelete,flagAdd,setFlagAdd }) => {
  const [rows, setRows] = useState<Point[]>([]);
  const [selectedName, setSelectedName] = useState<string | null>(null);
  const [puntos, setPuntos] = useState<{ nombre: string }[]>([
    { nombre: 'Rutina A' },
    { nombre: 'Rutina B' },
    { nombre: 'Rutina C' },
    { nombre: 'Rutina 1' },
    { nombre: 'Rutina 2' },
    { nombre: 'Rutina 3' },
    { nombre: 'Rutina 4' },
    { nombre: 'Rutina 5' },
    { nombre: 'Rutina 6' },
    { nombre: 'Rutina 7' },
    { nombre: 'Rutina 8' },
    { nombre: 'Rutina 9' },
    { nombre: 'Rutina 0' },
    { nombre: 'Rutina 10' },
    { nombre: 'Rutina 11' },
    { nombre: 'Rutina 12' },
    { nombre: 'Rutina 31' },
    { nombre: 'Rutina B11' },
    { nombre: 'Rutina C54' },
    { nombre: 'Rutina A55' },
    { nombre: 'Rutina B77' },
    { nombre: 'Rutina C5' },
  ]);
  
  
    useEffect(() => {
      nombrePuntoDB((msg) => {
        const nuevosPuntos = msg.nombres.map((nombre: string) => ({
          nombre,
        }));
        setPuntos(nuevosPuntos);
      });
  
  }, []);

  // Función para agregar puntos
  useEffect(() => {
    setRows(puntos.map((p) => ({ nombre: p.nombre })));
    setSelectedName(null);
  }, [puntos]);

  // Eliminar todos los puntos
  const eliminarTodo = () => {
    setRows([]);
    setSelectedName(null);
  };

  useEffect(() => {
    if (flagDelete) {
      eliminarTodo();
      setFlagDelete(false); // pero esto solo si le pasas setter del padre
    }
  }, [flagDelete, setFlagDelete]);

  useEffect(() => {
    if (flagAdd) {
      const nuevosPuntos=[
      { nombre: 'Rutina A' },
      { nombre: 'Rutina B' },
      { nombre: 'Rutina C' },
      { nombre: 'Rutina A' },
      { nombre: 'Rutina B' },
      { nombre: 'Rutina C' },
      { nombre: 'Rutina A' },
      { nombre: 'Rutina B' },
      { nombre: 'Rutina C' },
      { nombre: 'Rutina A' }]
      setPuntos(nuevosPuntos);
      setFlagAdd(false); // pero esto solo si le pasas setter del padre
    }
  }, [flagAdd, setFlagAdd]);

  // Seleccionar una fila
  const handleRowClick = (nombre: string) => {
    setSelectedName(nombre);
    onSelect(nombre);
  };

  return (
    <TableContainer  sx={{ maxHeight: "100%" }}>
      <Table size="small" stickyHeader>
        <colgroup>
          <col style={{ width: '90' }} />
        </colgroup>
        <TableHead>
          <TableRow>
            <TableCell sx={{ padding: "2px 8px" }}>Rutinas</TableCell>
          </TableRow>
        </TableHead>
        <TableBody>
          {rows.map((row) => (
            <TableRow
              onClick={() => handleRowClick(row.nombre)}
              selected={selectedName === row.nombre}
              hover
              sx={{ cursor: 'pointer' }}
            >
              <TableCell sx={{ padding: "2px 8px" }}>{row.nombre}</TableCell>
            </TableRow>
          ))}
        </TableBody>
      </Table>
    </TableContainer>
  );
};

export default RutineLibrarie;
