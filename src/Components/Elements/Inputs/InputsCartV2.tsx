import React, { useImperativeHandle, forwardRef, useState } from 'react';
import { Grid, Stack } from '@mui/material';
import Button1 from '../Buettons/Button1';
import CoordInputV2 from './CoordInputV2';
import { refreshCordCart } from "../../../Services/Funciones";

export type InputsCartVRef = {
  getValues: () => number[];
  setValues: (newVals: number[]) => void; 
};

type InputsCartVProps = {
  disabled?: boolean; 
};

const InputsCartV = forwardRef<InputsCartVRef, InputsCartVProps>(({ disabled = false }, ref) => {
  const [values, setValues] = useState<number[]>(Array(6).fill(0));

  const handleChange = (index: number) => (val: number) => {
    setValues((prev) => {
      const newVals = [...prev];
      newVals[index] = val;
      return newVals;
    });
  };
  
const handleBlurGlobal = (newVal: number, idx: number) => {
  setValues(prev => {
    const newVals = [...prev];
    newVals[idx] = newVal;
    refreshCordCart(newVals);
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
          {['x', 'y', 'z'].map((e, idx) => (
            <CoordInputV2
                key={idx}
                label={`cord ${e}`}        // "cord x", "cord y", etc.
                valMin={-1000}
                valMax={1000}
                value={values[idx]}        // usamos el índice para acceder al valor
                onChange={handleChange(idx)}
                disabled={disabled}
                onBlurCustom={(newVal: number) => handleBlurGlobal(newVal, idx)}
            />
          ))}
        </Stack>
      </Grid>

      <Grid size={{xs: 6, md: 6}}>
        <Stack spacing={1}>
          {['qx', 'qy', 'qz'].map((e, idx) => (
            <CoordInputV2
              key={idx+3}
              label={e}
              valMin={-1000}
              valMax={1000}
              value={values[idx+3]}
              onChange={handleChange(idx+3)}
              disabled={disabled}
              onBlurCustom={(newVal: number) => handleBlurGlobal(newVal, idx+3)}
            />
          ))}
        </Stack>
      </Grid>
    </Grid>
  );
});

export default InputsCartV;
