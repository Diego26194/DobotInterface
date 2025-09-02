import React, { useImperativeHandle, forwardRef, useState } from 'react';
import { Grid, Stack } from '@mui/material';
import Button1 from '../Buettons/Button1';
import InputCord from './CoordInput';

export type InputsCartsRef = {
  getValues: () => number[];
  setValues: (newVals: number[]) => void; 
};

type InputsCartsProps = {
  disabled?: boolean; 
};

const InputsCarts = forwardRef<InputsCartsRef, InputsCartsProps>(({ disabled = false }, ref) => {
  const [values, setValues] = useState<number[]>(Array(5).fill(0));

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
          {['x', 'y', 'z'].map((e, idx) => (
            <InputCord
                key={idx}
                label={`cord ${e}`}        // "cord x", "cord y", etc.
                valMin={0}
                valMax={180}
                value={values[idx]}        // usamos el índice para acceder al valor
                onChange={handleChange(idx)}
                disabled={disabled}
            />
          ))}
        </Stack>
      </Grid>

      <Grid size={{xs: 6, md: 6}}>
        <Stack spacing={1}>
          {['q', 'rad'].map((e, idx) => (
            <InputCord
              key={idx+3}
              label={e}
              valMin={0}
              valMax={180}
              value={values[idx+3]}
              onChange={handleChange(idx+3)}
              disabled={disabled}
            />
          ))}
        </Stack>
      </Grid>
    </Grid>
  );
});

export default InputsCarts;
