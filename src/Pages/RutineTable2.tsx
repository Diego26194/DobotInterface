import React, { useState } from "react";
import Box from "@mui/material/Box";
import { DataGrid, GridColDef } from "@mui/x-data-grid";

import { Select, MenuItem, TextField, SelectChangeEvent, IconButton } from "@mui/material";

import EditIcon from "@mui/icons-material/Edit";
import DoneIcon from "@mui/icons-material/Done";

// Opciones Select
const opcionesTrayectoria = ["Lineal", "Circular", "Mixta"];

// Celda Select personalizada
const SelectTrayectoriaCell = (params: any) => {
  const { id, field, value, api } = params;

  const handleChange = (event: SelectChangeEvent) => {
    api.setEditCellValue({ id, field, value: event.target.value }, event);
    api.commitCellChange({ id, field });
    api.setCellMode(id, field, "view");
  };

  return (
    <Select
      autoFocus
      value={value || ""}
      onChange={handleChange}
      fullWidth
      size="small"
    >
      {opcionesTrayectoria.map((opcion) => (
        <MenuItem key={opcion} value={opcion}>
          {opcion}
        </MenuItem>
      ))}
    </Select>
  );
};

const RutineTable = () => {
  // 👇 ESTADO dentro del componente
  const [rows, setRows] = useState([
    { id: 1, nombre: "Inicio", coordenadas: [0, 0, 0, 0, 0, 0], trayectoria: "Lineal", editable: false },
    { id: 2, nombre: "Punto A", coordenadas: [10, 20, 30, 0, 0, 0], trayectoria: "Circular", editable: false },
    { id: 3, nombre: "Punto B", coordenadas: [15, 25, 35, 0, 10, 0], trayectoria: "Mixta", editable: false },
  ]);

  // 👇 Funciones también dentro
  const handleEdit = (id: number) => {
    setRows((prev) =>
      prev.map((row) => (row.id === id ? { ...row, editable: true } : row))
    );
  };

  const handleConfirm = (id: number) => {
    setRows((prev) =>
      prev.map((row) => (row.id === id ? { ...row, editable: false } : row))
    );

    // Acá podrías mandar a ROS 👇
    const confirmedRow = rows.find((r) => r.id === id);
    console.log("Confirmado:", confirmedRow);
  };

  // 👇 Definimos columnas acá (así pueden acceder a las funciones)
  const columns: GridColDef[] = [
    { field: "id", headerName: "ID", width: 60 },
    { field: "nombre", headerName: "Nombre", width: 120, editable: true },
    {
      field: "coordenadas",
      headerName: "Coordenadas",
      width: 360,
      editable: true,
      renderEditCell: ({ id, field, value, api }) => {
        const handleChange = (index: number) => (e: React.ChangeEvent<HTMLInputElement>) => {
          const newCoords = [...value];
          newCoords[index] = parseFloat(e.target.value) || 0;
          api.setEditCellValue({ id, field, value: newCoords });
        };

        return (
          <Box sx={{ display: "flex", gap: 0.5 }}>
            {value.map((coord: number, i: number) => (
              <TextField
                key={i}
                type="number"
                value={coord}
                onChange={handleChange(i)}
                size="small"
                sx={{ width: 50 }}
              />
            ))}
          </Box>
        );
      },
      renderCell: (params) => <span>{params.value?.join(", ")}</span>,
    },
    {
      field: "trayectoria",
      headerName: "Trayectoria",
      width: 110,
      editable: true,
      renderEditCell: (params) => <SelectTrayectoriaCell {...params} />,
    },
    {
      field: "actions",
      headerName: "Acciones",
      width: 150,
      renderCell: (params) => {
        const rowEditable = params.row.editable;

        return rowEditable ? (
          <IconButton onClick={() => handleConfirm(params.row.id)} color="success">
            <DoneIcon />
          </IconButton>
        ) : (
          <IconButton onClick={() => handleEdit(params.row.id)} color="primary">
            <EditIcon />
          </IconButton>
        );
      },
    },
  ];

  return (
    <Box sx={{ height: "100%", width: "100%" }}>
      <DataGrid
        rows={rows}
        columns={columns}
        disableRowSelectionOnClick
        rowHeight={30}
        hideFooter
        isCellEditable={(params) => params.row.editable === true}
        sx={{
          border: "1px solid #000",
          "& .MuiDataGrid-cell": { fontSize: "0.75rem", py: 0 },
          "& .MuiDataGrid-columnHeaders": { minHeight: 32, maxHeight: 32 },
        }}
      />
    </Box>
  );
};

export default RutineTable;
