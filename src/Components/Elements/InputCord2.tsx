import React from "react";
//import TextField from "@mui/material/TextField";
import { styled } from "@mui/material/styles";

import CompactsInputs from "./Inputs/CompactsInput";

const NoSpinnerInput = styled(CompactsInputs)(({ theme }) => ({
  "& input::-webkit-outer-spin-button, & input::-webkit-inner-spin-button": {
    WebkitAppearance: "none",
    margin: 0,
  },
  "& input[type=number]": {
    MozAppearance: "textfield", // Firefox
  },
}));

type InputCordProps = {
  label: string;
  valMin: number;
  valMax: number;
  value: number;
  onChange: (val: number) => void;
  disabled?: boolean;
};

const InputCord: React.FC<InputCordProps> = ({ label, valMin, valMax, value, onChange, disabled = false }) => {
  const [inputVal, setInputVal] = React.useState(value.toString());

  const handleChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    let val = e.target.value;

    // Eliminar cualquier caracter que no sea dígito o el signo negativo al principio
    val = val.replace(/[^0-9-]/g, "");  // elimina puntos, comas, letras, etc.

    // Asegurar que solo haya un '-' y esté al principio
    if (val.includes("-")) {
      val = val.replace(/(?!^)-/g, ""); 
    }

    setInputVal(val);

    const num = parseInt(val, 10);
    if (!isNaN(num)) {
      onChange(num);
    }
  };

  const numericVal = parseInt(inputVal, 10);
  const isOutOfRange = !isNaN(numericVal) && (numericVal < valMin || numericVal > valMax);

  return (
    <NoSpinnerInput
      type="number"
      label={label}
      value={inputVal}
      onChange={handleChange}
      disabled={disabled}
      inputProps={{
        inputMode: "numeric", // antes estaba "decimal"
        pattern: "-?[0-9]*",  // solo dígitos, con signo negativo opcional
      }}
      error={isOutOfRange}
      /*
      helperText={
        isOutOfRange
          ? `Debe estar entre ${valMin} y ${valMax}`
          : "Ingrese un número con hasta 2 decimales"
      }
          */
    />
  );
};

export default InputCord;
