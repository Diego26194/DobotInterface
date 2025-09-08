import React, { useState, useEffect, useRef, forwardRef, useImperativeHandle } from "react";
import Box from '@mui/material/Box';
import { DataGrid, GridColDef, GridRowId, GridRowSelectionModel } from '@mui/x-data-grid';


import { Select, MenuItem, TextField } from '@mui/material';
import { SelectChangeEvent, IconButton } from '@mui/material';

import EditIcon from "@mui/icons-material/Edit";
import DoneIcon from "@mui/icons-material/Done";
import PlayArrowIcon from '@mui/icons-material/PlayArrow';

import { resibirMsgRutina ,elimiarPuntoRutina, correrTAngular, msgEmergente } from "../Services/Funciones";
import InputPoisitive from "../Components/Elements/Inputs/InputPoisitive";


//COLUMNA Select

const opcionesTrayectoria = ['PTP', 'LIN', 'CIRC'];

const SelectTrayectoriaCell = (params: any) => {
  const { id, field, value, api } = params;

  const handleChange = (event: SelectChangeEvent) => {
    api.setEditCellValue({ id, field, value: event.target.value }, event);
    api.commitCellChange({ id, field });
    api.setCellMode(id, field, 'view');
  };

  return (
    <Select
      autoFocus
      value={value || ''}
      onChange={handleChange}
      fullWidth
      size="small"
      sx={{
        width: "100%",
        fontSize: "0.75rem",
        padding: "0px 1px",
        minHeight: "20px",
        maxHeight: "26px",
        textAlign: "center",
      }}
    >
      {opcionesTrayectoria.map((opcion) => (
        <MenuItem 
          key={opcion} 
          value={opcion} 
          sx={{
            width: "100%",
            fontSize: "0.75rem",
            padding: "0px 12px",
            minHeight: "24px",
            textTransform: "none",
            justifyContent: "center",
          }}>
          {opcion}
        </MenuItem>
      ))}
    </Select>
  );
};

interface NumericEditCellProps {
  id: GridRowId;
  field: string;
  value?: number;
  api: any;
}

const NumericEditCell: React.FC<NumericEditCellProps> = ({ id, field, value, api }) => {
  const handleChange = (newValue: number) => {
    api.setEditCellValue({ id, field, value: newValue });
  };

  return (
    <InputPoisitive
      value={value ?? 0}
      valMin={0}
      valMax={100}
      onChange={handleChange}
      variant="outlined"
      sx={{'& .MuiInputBase-root': {
          height: 20,           // Igual a tu rowHeight
          fontSize: '0.75rem',  // Tamaño legible
          padding: '0 4px',     // Pequeño padding horizontal
        },
        '& input': {
          height: '100%',       // Para que ocupe todo el contenedor
          padding: 0,           // Sin padding extra
          fontSize: '0.75rem',  // Mismo tamaño de font
        },
      }}
    />
  );
};

type RowType = {
  id: number; 
  posicion: number;           
  nombre: string;
  coordenadas: number[]; // 6 elementos
  escV: number; 
  ratio: number; 
  plan: string;
  editable: boolean;
  wait: number; 
  rutine: boolean;
};


interface RutineTableProps {
  flagEliminarPRut: boolean;
  setFlagEliminarPRut: React.Dispatch<React.SetStateAction<boolean>>;
}

export type RutineTableRef = {
  updateWaitForSelectedRows: (valor: number) => void;
};
const RutineTable = forwardRef<RutineTableRef,RutineTableProps> (({flagEliminarPRut, setFlagEliminarPRut}, ref) => {

//const RutineTable = () => {
  const [rowSelectionModel, setRowSelectionModel] =
    React.useState<GridRowSelectionModel>({ type: 'include', ids: new Set() });
  const [rows, setRows] = useState([
    { id: 1, posicion: 1 , nombre: 'Inicio', coordenadas: [0, 0, 0, 0, 0, 0], escV: 10, ratio:0, plan: 'PTP' , editable: false, wait: 0, rutine: false },
    { id: 2, posicion: 2 , nombre: 'Punto A', coordenadas: [10, 20, 30, 0, 0, 0], escV: 50, ratio:50, plan: 'LIN' , editable: false, wait: 3, rutine: false },
    { id: 3, posicion: 3, nombre: 'Ir a Torno', coordenadas: [15, 25, 35, 0, 10, 0], escV: 100, ratio:0, plan: 'CIRC' , editable: false, wait: 0 , rutine: true},
    { id: 4, posicion: 4 , nombre: 'Punto B', coordenadas: [10, 20, 30, 0, 0, 0], escV: 50, ratio:50, plan: 'LIN' , editable: false, wait: 3, rutine: false },
  ]);
    //const [wite, setShoWite] = useState(0);

  useEffect(() => {
    if (flagEliminarPRut) {
      const idsArray = Array.from(rowSelectionModel.ids);
      const nombresArray = rows
      .filter((row) => idsArray.includes(row.id)) 
      .map((row) => row.nombre);
      
      if (idsArray.length > 0) {
        elimiarPuntoRutina(nombresArray, idsArray.map(Number));
        //eliminarInstruccionRutina(idsArray.map(Number),['delPR','Punto A','Ir a Torno']);
        eliminarInstruccionRutina(idsArray.map(Number));
        msgEmergente('ErrordelPR');
      }

      setFlagEliminarPRut(false);
    }
  }, [flagEliminarPRut, rowSelectionModel, setFlagEliminarPRut]);

  useEffect(() => {
    resibirMsgRutina((msg) => {
      switch (msg.orden[0]) {
        case 'addP':
          addRowPunto(msg.orden, msg.coordenadas);
          break;

        case 'errorP':
          msgEmergente('errorP');
          break;

        case 'delPR':
          eliminarInstruccionRutina(msg.coordenadas);
          break;

        case 'ErrordelPR':
          msgEmergente('ErrordelPR');
          break;

        case 'errorR':
          msgEmergente('errorR');
          break;

        default:
          console.warn("Acción no reconocida:", msg.orden[0]);
      }
    });
  }, []);

  useImperativeHandle(ref, () => ({
    updateWaitForSelectedRows,
  }));

  const updateWaitForSelectedRows = (newWait: number) => {
    console.log(newWait);
    setRows((prevRows) =>
      prevRows.map((row) =>
        rowSelectionModel.ids.has(row.id) ? { ...row, wait: newWait } : row
      )
    );
    console.log(rows);
  };
  
  const addRowPunto = (orden: string[], coordenadas: number[]) => { //orden [id,noombre,plan] coordenadas[a1,a2,a3,a4,a5,a6,vel, ratio]
    setRows((prev) => {
      const newRow: RowType = {
        id: Date.now(),      
        posicion: coordenadas[0], //prev.length + 1,         
        nombre: orden[1],
        plan: orden[2],
        coordenadas: coordenadas.slice(1, 7),
        escV: coordenadas[7], 
        ratio: coordenadas[8], 
        editable: false,
        wait: 0,
        rutine: false,                   
      };
      return [...prev, newRow];
    });
  };  
   
  const eliminarInstruccionRutina = (posiciones: number[]) => {
    setRows((prevRows) => {
      // 1. Filtrar las filas que NO están en posiciones a eliminar
      const filtradas = prevRows.filter(row => !posiciones.includes(row.posicion));

      // 2. Recalcular posiciones
      return filtradas.map((row, index) => ({
        ...row,
        posicion: index + 1
      }));
    });
  };
{/* 
  const FalloEliminarIRutina=(posiciones: number[])=> {
    const mensaje = [
      "              ⚠ No se elimino nunguna fila:",
      " Las siguientes instrucciones no se encuentran en la rutina o no coinciden con la base de datos:",
      ...posiciones.map(
        (item) =>
          `- Posición ${item}`,
      ),
      "",
      "🔄 Refresque la tabla para asegurarse de trabajar con los datos reales.",
    ].join("\n");

    alert(mensaje);
  }
  */}
  {/* 
  const eliminarInstruccionRutina = (posiciones: number[], orden: string[]) => {
  setRows((prevRows: RowType[]) => {
    // Relacionar posiciones con nombres esperados
    const objetivos = posiciones.map((pos, i) => ({
      pos,
      nombreEsperado: orden[i + 1],
    }));

    const noCoinciden: { pos: number; esperado: string; encontrado?: string }[] = [];

    // Filtrar filas
    const filtradas = prevRows.filter((row) => {
      const objetivo = objetivos.find((o) => o.pos === row.posicion);
      if (!objetivo) return true; // no estaba en la lista → se queda

      if (row.nombre === objetivo.nombreEsperado) {
        return false; // ✅ coincide, se elimina
      } else {
        // ❌ no coincide → lo guardamos como error
        noCoinciden.push({
          pos: row.posicion,
          esperado: objetivo.nombreEsperado,
          encontrado: row.nombre,
        });
        return true; // no lo eliminamos
      }
    });

    // Mostrar popup si hubo inconsistencias
    if (noCoinciden.length > 0) {
      const mensaje = [
        "⚠ Los siguientes puntos no se encuentran en la rutina o no coinciden con la base de datos:",
        ...noCoinciden.map(
          (item) =>
            `- Posición ${item.pos}: esperado "${item.esperado}" pero encontrado "${item.encontrado ?? "ninguno"}"`
        ),
        "",
        "🔄 Refresque la tabla para asegurarse de trabajar con los datos reales.",
      ].join("\n");

      alert(mensaje);
    }    

    // Recalcular posiciones
    return filtradas.map((row, index) => ({
      ...row,
      posicion: index + 1,
    }));
  });
};
*/}

  
  const handleEdit = (id: number) => {
    setRows((prev) =>
      prev.map((row) =>
        row.id === id ? { ...row, editable: true } : row
      )
    );
   // const pruebaID = mandarIDSeleccionados();
    //console.log(" los numeros ganadores son", pruebaID);
    //console.log(" los numeros ganadores son", rowSelectionModel);
  };

  const handleConfirm = (id: number) => {
    setRows((prev) =>
      prev.map((row) =>
        row.id === id ? { ...row, editable: false } : row
      )
    );
  };

  const handlePlayRow = (rowId: number) => {
    // Buscamos la fila correspondiente en el estado `rows`
    const row = rows.find(r => r.id === rowId);
    if (row) {
      console.log("Coordenadas:", row.coordenadas);
      alert(`Coordenadas: [${row.coordenadas.join(', ')}]`);
      correrTAngular(row.coordenadas)
    }
  };

  const columns: GridColDef[] = [
    { field: 'posicion', headerName: 'Pos.', width: 40 },
    {
      field: 'nombre',
      headerName: 'Nombre',
      width: 100,
      editable: true,
    },
  {
      field: 'coordenadas',
      headerName: 'Coordenadas',
      width: 330,
      editable: true,  
      renderEditCell: ({ id, field, value, api }) => {
        const handleChange = (index: number) => (e: React.ChangeEvent<HTMLInputElement>) => {
          const newCoords = [...value];
          newCoords[index] = parseFloat(e.target.value) || 0;
          api.setEditCellValue({ id, field, value: newCoords });
        };

        return (
          <Box sx={{ display: 'flex', gap: 0.5 }}>
            {value.map((coord: number, i: number) => (
              <TextField
                key={i}
                type="number"
                value={coord}
                onChange={handleChange(i)}
                size="small"
                InputProps={{
                  inputProps: {
                    step: '1',
                    inputMode: 'numeric',
                  },
                }}
                sx={{
                  width: 50,
                  height: '100%',
                  /*
                  '& input::-webkit-outer-spin-button': {
                    display: 'none',
                  },
                  '& input::-webkit-inner-spin-button': {
                    display: 'none',
                  },*/
                  '& .MuiInputBase-root': {
                    fontSize: '0.75rem',
                    padding: '0px 4px',
                    minHeight: '2px',
                  },
                  '& .MuiInputLabel-root': {
                    fontSize: '0.70rem',
                  },
                  '& input': {
                    padding: 0,
                  },
                }}
              />
            ))}
          </Box>
        );
      },
      renderCell: (params) => params.row.rutine === false ? 
        (<span>{params.value?.join(', ')}</span>
      ) : null
    },
    {
      field: 'escV',
      headerName: 'Esc.V',
      type: 'number',
      width: 50,
      editable: true,
      renderEditCell: (params) => <NumericEditCell {...params} />,
      renderCell: (params) =>params.row.rutine === false ? params.row.escV : null
    },
  /*  {
      field: 'escA',
      headerName: 'Esc.A',
      type: 'number',
      width: 80,
      editable: true,
    },
    {
      field: 'radio',
      headerName: 'Radio',
      type: 'number',
      width: 80,
      editable: true,
    },
    */
    {
      field: 'ratio',
      headerName: 'Ratio',
      type: 'number',
      width: 50,
      editable: true,
      renderEditCell: (params) => <NumericEditCell {...params} />,
      renderCell: (params) =>params.row.rutine === false ? params.row.ratio : null
    },
    {
      field: 'plan',
      headerName: 'Plan',
      width: 80,
      editable: true,
      renderEditCell: (params) => <SelectTrayectoriaCell {...params} />,
      renderCell: (params) =>params.row.rutine === false ? params.row.plan : null
    },
    {
      field: "actions",
      headerName: "Edit",
      width: 40,
      renderCell: (params) => {

        if (params.row.rutine) return null;

        const rowEditable = params.row.editable;

        return rowEditable ? (
          <IconButton
            onClick={() => handleConfirm(params.row.id)}
            color="success"       
            sx={{ 
              height: '25%',              
              minHeight: '2px',
              "&:focus": { outline: "none" },   // saca el borde de focus
              "&:focus-visible": { outline: "none" },
            }}
          >
            <DoneIcon fontSize="small"/>
          </IconButton>
        ) : (
          <IconButton
            onClick={() => handleEdit(params.row.id)}
            color="primary"
            sx={{ 
              height: '25%',              
              minHeight: '2px',
              "&:focus": { outline: "none" },   // saca el borde de focus
              "&:focus-visible": { outline: "none" },
            }}
          >
            <EditIcon fontSize="small"/>
          </IconButton>
        );
      },
    },
    {
      field: "correr",
      headerName: "Correr",
      width: 40,
      renderCell: (params) => (
          <IconButton
            onClick={() => handlePlayRow(params.row.id)}
            color="primary"       
            sx={{ 
              height: '25%',              
              minHeight: '2px',
              "&:focus": { outline: "none" },   // saca el borde de focus
              "&:focus-visible": { outline: "none" },
            }}
          >
            <PlayArrowIcon fontSize="small"/>
          </IconButton>
        ),
    },
    {
      field: "wait",
      headerName: "Wait",
      type: 'number',
      width: 40,
      editable: true,
      renderEditCell: (params) => <NumericEditCell {...params} />,
      renderCell: (params) =>params.row.wait !== 0 ? params.row.wait : null
    }
  ];

  return (
    <Box sx={{ height: '100%', width: '100%'
      
    }}>
      <DataGrid
        rows={rows}
        columns={columns}
        checkboxSelection
        disableRowSelectionOnClick
        rowHeight={20}
        hideFooter
        isCellEditable={(params) => params.row.editable === true}
        processRowUpdate={(newRow, oldRow) => {
          // Acá actualizás el estado con los valores editados
          setRows((prev) =>
            prev.map((row) => (row.id === newRow.id ? { ...newRow } : row))
          );
          return newRow; // importante que retorne el nuevo row
        }}

        //rowSelectionModel={selectionModel}
        //onRowSelectionModelChange={(newModel) => {setSelectionModel(newModel);}}
        onRowSelectionModelChange={(newRowSelectionModel) => {
          setRowSelectionModel(newRowSelectionModel);
        }}
        rowSelectionModel={rowSelectionModel}

        sx={{
          border: '1px solid #000',
          maxHeight: "100%",
          '& .MuiDataGrid-cell': {
            fontSize: '0.75rem', // Tamaño de letra
            py: 0.0, // padding vertical reducido (altura de fila)
            
            maxHeight: 20,
            minHeight: 20,
          },
          '& .MuiDataGrid-columnHeaders': {
            minHeight: 32,   // Altura mínima total del encabezado
            maxHeight: 32,
          },
          '& .MuiDataGrid-columnHeader': {
            fontSize: '0.75rem',
            py: 0,
            maxHeight: 30,
            minHeight: 30,
          },
          '& .MuiDataGrid-row': {
            maxHeight: 30,
            minHeight: 30,
          },
        }}
      />
    </Box>
  );
});

export default RutineTable;
