import React, { useImperativeHandle, forwardRef, useState, useEffect } from 'react';
import { Grid, Stack } from '@mui/material';
import Button1 from './Buettons/Button1';
import CoordInput from './Inputs/CoordInput';

//import { cargarPuntosDB, elimiarPuntoDB, escucharPuntoDB } from "../../Services/Funciones";

export type InputsAngsRef = {
  getValues: () => number[];
  setValues: (newVals: number[]) => void; 
};

type InputsAngsProps = {
  disabled?: boolean; 
};

const InputsAngs = forwardRef<InputsAngsRef, InputsAngsProps>(({ disabled = false }, ref) => {
  const [values, setValues] = useState<number[]>(Array(6).fill(0));



  const handleChange = (index: number) => (val: number) => {
    
    setValues((prev) => {
      const newVals = [...prev];
      newVals[index] = val;
      return newVals;
    });
  };

  useImperativeHandle(ref, () => ({
    getValues: () => values,
    setValues: (newVals: number[]) => setValues(newVals)
  }));

  return (
    <Grid
      container
      spacing={2}
      sx={{
        width: '100%',
        padding: 1,
      }}
    >
      <Grid size={{xs: 6, md: 6}}>
        <Stack spacing={1}>
          {[0, 1, 2].map((i) => (
            <CoordInput
              key={i}
              label={`ang ${i + 1}`}
              valMin={0}
              valMax={180}
              value={values[i]}
              onChange={handleChange(i)}
              disabled={disabled}
            />
          ))}
        </Stack>
      </Grid>

      <Grid size={{xs: 6, md: 6}}>
        <Stack spacing={1}>
          {[3, 4, 5].map((i) => (
            <CoordInput
              key={i}
              label={`ang ${i + 1}`}
              valMin={0}
              valMax={180}
              value={values[i]}
              onChange={handleChange(i)}
              disabled={disabled}
            />
          ))}
        </Stack>
      </Grid>
    </Grid>
  );
});

export default InputsAngs;
