import React, { useState, useEffect } from 'react';
import {
  Table, TableBody, TableCell, TableContainer,
  TableHead, TableRow, Paper
} from '@mui/material';
import { nombreRutinaDB } from "../../Services/Funciones";

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
  const [puntos, setPuntos] = useState<{ nombre: string }[]>([]);
  
  
    useEffect(() => {
      nombreRutinaDB((msg) => {
        const nuevosPuntos = msg.nombres.map((nombre: string, index: number) => ({
          num: index,
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
