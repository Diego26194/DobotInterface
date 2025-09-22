import React, { useImperativeHandle, forwardRef, useState, useEffect } from 'react';
import { Grid, Stack } from '@mui/material';
import Button1 from '../Buettons/Button1';
import CoordInputV2 from './CoordInputV2';
import { refreshCordAng } from "../../../Services/Funciones";

//import { cargarPuntosDB, elimiarPuntoDB, escucharPuntoDB } from "../../Services/Funciones";

export type InputsAngV2Ref = {
  getValues: () => number[];
  setValues: (newVals: number[]) => void; 
};

type InputsAngV2Props = {
  disabled?: boolean; 
};

const InputsAngV2 = forwardRef<InputsAngV2Ref, InputsAngV2Props>(({ disabled = false }, ref) => {
  const [values, setValues] = useState<number[]>(Array(6).fill(0));



  const handleChange = (index: number) => (val: number) => {
    
    setValues((prev) => {
      const newVals = [...prev];
      newVals[index] = val;
      return newVals;
    });
  };

  const handleBlurGlobal = () => {
    refreshCordAng(values);  // siempre tiene los 6 valores actualizados
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
            <CoordInputV2
              key={i}
              label={`ang ${i + 1}`}
              valMin={0}
              valMax={180}
              value={values[i]}
              onChange={handleChange(i)}
              disabled={disabled}
              onBlurCustom={handleBlurGlobal}
            />
          ))}
        </Stack>
      </Grid>

      <Grid size={{xs: 6, md: 6}}>
        <Stack spacing={1}>
          {[3, 4, 5].map((i) => (
            <CoordInputV2
              key={i}
              label={`ang ${i + 1}`}
              valMin={0}
              valMax={180}
              value={values[i]}
              onChange={handleChange(i)}
              disabled={disabled}
              onBlurCustom={handleBlurGlobal}
            />
          ))}
        </Stack>
      </Grid>
    </Grid>
  );
});

export default InputsAngV2;
