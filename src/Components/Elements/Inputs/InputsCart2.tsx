import React, { useImperativeHandle, forwardRef, useState, Ref } from 'react';
import { Grid, Stack } from '@mui/material';
import Button1 from '../Buettons/Button1';
import InputCord from '../InputCord';

export type InputsCartsRef2 = {
  getValues: () => number[];
};
interface valorProps{
  valorsPrueba:Ref<any>
}

const InputsCarts2 :React.FC<valorProps>=({valorsPrueba}) => {
  const [values, setValues] = useState<number[]>(Array(6).fill(0));

  const handleChange = (index: number) => (val: number) => {
    setValues((prev) => {
      const newVals = [...prev];
      newVals[index] = val;
      return newVals;
    });
  };

  useImperativeHandle(valorsPrueba, () => ({
    getValues: () => values,
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
            />
          ))}
        </Stack>
      </Grid>
    </Grid>
  );
};

export default InputsCarts2;
