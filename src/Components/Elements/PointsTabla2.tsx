import React, { useState, useEffect, useImperativeHandle, forwardRef } from 'react';
import {
  Table, TableBody, TableCell, TableContainer,
  TableHead, TableRow, Paper
} from '@mui/material';
import { escucharPuntoDB } from "../../Services/Funciones";

interface Point {
  nombre: string;
}

interface PointsTableProps {
  onSelect: (nombre: string) => void;
}

export interface PointsTableRef {
  eliminarTodo: () => void;  // el padre podrá llamar a esto
}

interface PointsTableProps {
  onSelect: (nombre: string) => void;  // Para pasar el nombre al hacer clic
}

const PointsTable = forwardRef<PointsTableRef, PointsTableProps>( ({ onSelect }, ref) => {
  const [rows, setRows] = useState<Point[]>([]);
  const [selectedName, setSelectedName] = useState<string | null>(null);
  const [puntos, setPuntos] = useState<{ num?: number; nombre: string }[]>([
    { nombre: 'Punto A' },
    { nombre: 'Punto B' },
    { nombre: 'Punto C' },
    { nombre: 'Punto d' },
    { nombre: 'Punto e' },
    { nombre: 'Punto f' },
    { nombre: 'Punto g' },
    { nombre: 'Punto h' },
    { nombre: 'Punto i' },
    { nombre: 'Punto l' },
    { nombre: 'Punto m' },
    { nombre: 'Punto n' },
    { nombre: 'Punto 11' },
    { nombre: 'Punto 2' },
    { nombre: 'Punto 3' },
    { nombre: 'Punto 4' },
    { nombre: 'Punto 5' },
    { nombre: 'Punto 6' },
    { nombre: 'Punto 7' },
    { nombre: 'Punto 8' },
    { nombre: 'Punto C9' },
  ]);
  
  
  useEffect(() => {
    const stop = escucharPuntoDB((msg) => {
      if (!msg.coordenadas || msg.coordenadas.length === 0) {
        //eliminarTodo();
        const nuevosPuntos = msg.orden.map((nombre: string, index: number) => ({
          num: index,
          nombre,
        }));
        setPuntos(nuevosPuntos);
      }
    });
    }, []);

  // Función para agregar puntos
  useEffect(() => {
    setRows(puntos.map((p) => ({ nombre: p.nombre })));
    setSelectedName(null);
  }, [puntos]);

  // Eliminar todos los puntos
  const eliminarTodo = () => {
    console.log('Se Borro?')
    setRows([]);
    setSelectedName(null);
  };

  // Seleccionar una fila
  const handleRowClick = (nombre: string) => {
    setSelectedName(nombre);
    onSelect(nombre);
  };

   // Exponer función al padre
    useImperativeHandle(ref, () => ({
      eliminarTodo,
    }));

  return (
    <TableContainer component={Paper} sx={{ maxHeight: "100%" }}>
      <Table size="small" stickyHeader>
        <colgroup>
          <col style={{ width: '10%' }} />
          <col style={{ width: '90%' }} />
        </colgroup>
        <TableHead>
          <TableRow>
            <TableCell sx={{ padding: "2px 8px" }}>Num</TableCell>
            <TableCell sx={{ padding: "2px 8px" }}>Nombre</TableCell>
          </TableRow>
        </TableHead>
        <TableBody>
          {rows.map((row, index) => (
            <TableRow
              key={index}
              onClick={() => handleRowClick(row.nombre)}
              selected={selectedName === row.nombre}
              hover
              sx={{ cursor: 'pointer' }}
            >
              <TableCell sx={{ padding: "2px 8px" }}>{index + 1}</TableCell>
              <TableCell sx={{ padding: "2px 8px" }}>{row.nombre}</TableCell>
            </TableRow>
          ))}
        </TableBody>
      </Table>
    </TableContainer>
  );
});

export default PointsTable;
