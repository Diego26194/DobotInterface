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

  const handleChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    let val = e.target.value;

    // Eliminar cualquier caracter que no sea dígito o el signo negativo al principio
    val = val.replace(/[^0-9-]/g, "");  // elimina puntos, comas, letras, etc.

    // Asegurar que solo haya un '-' y esté al principio
    if (val.includes("-")) {
      val = val.replace(/(?!^)-/g, ""); 
    }

    if (val === "" || val === "-") {
      onChange(NaN); // usamos NaN para indicar input "vacío" o solo "-"
      return;
    }    

    const num = parseInt(val, 10);
    if (!isNaN(num)) {
      onChange(num);
    } 
  };
  const handleBlur = () => {
    if (isNaN(value)) onChange(0);
  };

  const numericVal = parseInt(value.toString(), 10);
  const isOutOfRange = !isNaN(numericVal) && (numericVal < valMin || numericVal > valMax);

  return (
    <NoSpinnerInput
      type="number"
      label={label}
      value={value.toString()}   
      onChange={handleChange}
      disabled={disabled}
      inputProps={{
        inputMode: "numeric",
        pattern: "-?[0-9]*",
      }}
      error={isOutOfRange}
      onBlur={handleBlur}
    />
  );
};

export default InputCord;
